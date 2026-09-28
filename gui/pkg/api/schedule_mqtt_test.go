package api

import (
	"encoding/json"
	"os"
	"path/filepath"
	"sync"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// fakeMqttClient is the test double for mqttClient — records every publish and
// lets a test invoke a subscribed handler directly, with no real broker.
type fakeMqttClient struct {
	mu           sync.Mutex
	published    []fakePublication
	handlers     map[string]func(payload []byte)
	disconnected bool
}

type fakePublication struct {
	topic    string
	payload  []byte
	retained bool
}

func newFakeMqttClient() *fakeMqttClient {
	return &fakeMqttClient{handlers: map[string]func(payload []byte){}}
}

func (f *fakeMqttClient) Publish(topic string, payload []byte, retained bool) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.published = append(f.published, fakePublication{topic, payload, retained})
	return nil
}

func (f *fakeMqttClient) Subscribe(topic string, handler func(payload []byte)) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.handlers[topic] = handler
	return nil
}

func (f *fakeMqttClient) Disconnect() { f.disconnected = true }

func (f *fakeMqttClient) fire(t *testing.T, topic string, payload []byte) {
	t.Helper()
	f.mu.Lock()
	h, ok := f.handlers[topic]
	f.mu.Unlock()
	require.True(t, ok, "no handler subscribed for %s", topic)
	h(payload)
}

func (f *fakeMqttClient) lastPublished(topic string) (fakePublication, bool) {
	f.mu.Lock()
	defer f.mu.Unlock()
	for i := len(f.published) - 1; i >= 0; i-- {
		if f.published[i].topic == topic {
			return f.published[i], true
		}
	}
	return fakePublication{}, false
}

func (f *fakeMqttClient) publishCount() int {
	f.mu.Lock()
	defer f.mu.Unlock()
	return len(f.published)
}

// ===========================================================================
// Publishing the schedule list
// ===========================================================================

func TestScheduleMqttBridge_PublishesCurrentSchedulesOnStart(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, saveSchedule(db, &Schedule{ID: "1", Area: 2, Time: "07:30", DaysOfWeek: []int{1, 3}, Enabled: true}))

	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	pub, ok := client.lastPublished("mowgli/schedules")
	require.True(t, ok)
	assert.True(t, pub.retained, "the schedule mirror must be retained, like <prefix>/area_boundary")

	var resp ScheduleListResponse
	require.NoError(t, json.Unmarshal(pub.payload, &resp))
	require.Len(t, resp.Schedules, 1)
	assert.Equal(t, "1", resp.Schedules[0].ID)
	assert.Equal(t, 2, resp.Schedules[0].Area)
}

func TestScheduleMqttBridge_PublishesEmptyArrayNotNullWhenThereAreNone(t *testing.T) {
	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	pub, ok := client.lastPublished("mowgli/schedules")
	require.True(t, ok)
	assert.JSONEq(t, `{"schedules":[]}`, string(pub.payload))
}

// ===========================================================================
// schedules/set: create and update
// ===========================================================================

func TestScheduleMqttBridge_SetWithNoIdCreatesASchedule(t *testing.T) {
	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/set", []byte(`{"area":0,"time":"06:00","daysOfWeek":[1,2,3,4,5],"enabled":true}`))

	schedules, err := getAllSchedules(db)
	require.NoError(t, err)
	require.Len(t, schedules, 1)
	assert.NotEmpty(t, schedules[0].ID, "a generated id, same as the HTTP POST path")
	assert.Equal(t, "06:00", schedules[0].Time)
	assert.False(t, schedules[0].CreatedAt.IsZero())
}

func TestScheduleMqttBridge_SetWithAnExistingIdUpdatesAndKeepsHistory(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, saveSchedule(db, &Schedule{
		ID: "42", Area: 1, Time: "06:00", DaysOfWeek: []int{1}, Enabled: true,
		LastSkipReason: "soil wet",
	}))
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/set", []byte(`{"id":"42","area":1,"time":"08:00","daysOfWeek":[1,2],"enabled":false}`))

	updated, err := getSchedule(db, "42")
	require.NoError(t, err)
	assert.Equal(t, "08:00", updated.Time)
	assert.False(t, updated.Enabled)
	assert.Equal(t, "soil wet", updated.LastSkipReason, "history fields survive an MQTT update, like the HTTP PUT path")
}

func TestScheduleMqttBridge_SetRejectsAnInvalidScheduleAndDoesNotSaveIt(t *testing.T) {
	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)
	before := client.publishCount()

	client.fire(t, "mowgli/schedules/set", []byte(`{"area":0,"time":"25:99","daysOfWeek":[1]}`))

	schedules, err := getAllSchedules(db)
	require.NoError(t, err)
	assert.Empty(t, schedules, "the same validateSchedule the HTTP API uses must reject this")
	assert.Equal(t, before, client.publishCount(), "a rejected write must not republish")
}

func TestScheduleMqttBridge_SetPublishesTheUpdatedList(t *testing.T) {
	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/set", []byte(`{"area":0,"time":"06:00","daysOfWeek":[1]}`))

	pub, ok := client.lastPublished("mowgli/schedules")
	require.True(t, ok)
	var resp ScheduleListResponse
	require.NoError(t, json.Unmarshal(pub.payload, &resp))
	require.Len(t, resp.Schedules, 1)
}

// ===========================================================================
// schedules/delete
// ===========================================================================

func TestScheduleMqttBridge_DeleteAcceptsAPlainIdPayload(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, saveSchedule(db, &Schedule{ID: "7", Area: 0, Time: "06:00", DaysOfWeek: []int{1}}))
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/delete", []byte("7"))

	_, err := getSchedule(db, "7")
	assert.Error(t, err)
}

func TestScheduleMqttBridge_DeleteAcceptsAJsonIdPayload(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, saveSchedule(db, &Schedule{ID: "7", Area: 0, Time: "06:00", DaysOfWeek: []int{1}}))
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/delete", []byte(`{"id":"7"}`))

	_, err := getSchedule(db, "7")
	assert.Error(t, err)
}

func TestScheduleMqttBridge_DeleteWithNoIdIsIgnored(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, saveSchedule(db, &Schedule{ID: "7", Area: 0, Time: "06:00", DaysOfWeek: []int{1}}))
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/delete", []byte(`{}`))

	_, err := getSchedule(db, "7")
	assert.NoError(t, err, "an empty payload must not be treated as \"delete everything\"")
}

// ===========================================================================
// The HTTP API (the GUI's own Schedules page) also reaches MQTT
// ===========================================================================

func TestScheduleMqttBridge_AnHttpCreateAlsoRepublishesToMqtt(t *testing.T) {
	// registerScheduleChangeListener is process-global (mirrors how schedules.go
	// has no import of this file); clear it so an earlier test's bridge isn't
	// still registered and asserting against the wrong fake client.
	scheduleChangeListenersMu.Lock()
	scheduleChangeListeners = nil
	scheduleChangeListenersMu.Unlock()

	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)
	before := client.publishCount()

	sched := Schedule{Area: 0, Time: "06:00", DaysOfWeek: []int{1}, Enabled: true}
	require.NoError(t, validateSchedule(&sched))
	sched.ID = "999"
	require.NoError(t, saveSchedule(db, &sched))
	notifyScheduleChanged() // what createSchedule's HTTP handler calls after saving

	assert.Greater(t, client.publishCount(), before)
	pub, _ := client.lastPublished("mowgli/schedules")
	var resp ScheduleListResponse
	require.NoError(t, json.Unmarshal(pub.payload, &resp))
	require.Len(t, resp.Schedules, 1)
	assert.Equal(t, "999", resp.Schedules[0].ID)
}

// ===========================================================================
// Broker settings resolution
// ===========================================================================

func writeYamlConfig(t *testing.T, db *types.MockDBProvider, body string) {
	t.Helper()
	path := filepath.Join(t.TempDir(), "mowgli_robot.yaml")
	require.NoError(t, os.WriteFile(path, []byte(body), 0o600))
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(path)))
}

func TestLoadMqttBrokerSettings_ReadsExplicitValues(t *testing.T) {
	db := types.NewMockDBProvider()
	writeYamlConfig(t, db, `
mowgli:
  ros__parameters:
    mqtt_enabled: true
    mqtt_host: "broker.lan"
    mqtt_port: 8883
    mqtt_username: "arco"
    mqtt_password: "secret"
    mqtt_topic_prefix: "garden"
`)

	s, err := loadMqttBrokerSettings(db)
	require.NoError(t, err)
	assert.Equal(t, mqttBrokerSettings{
		enabled: true, host: "broker.lan", port: 8883,
		username: "arco", password: "secret", topicPrefix: "garden",
	}, s)
}

func TestLoadMqttBrokerSettings_AbsentKeysFallBackToTheSameDefaultsAsTheTemplate(t *testing.T) {
	// Isolate from any schema cached by another test in this package (getSchema
	// caches process-wide); this test only exercises the hardcoded fallback
	// that applies when the schema itself carries no default either.
	schemaCacheMu.Lock()
	savedSchema, savedTime := schemaCache, schemaCacheTime
	schemaCache, schemaCacheTime = map[string]any{}, savedTime
	schemaCacheMu.Unlock()
	t.Cleanup(func() {
		schemaCacheMu.Lock()
		schemaCache, schemaCacheTime = savedSchema, savedTime
		schemaCacheMu.Unlock()
	})

	db := types.NewMockDBProvider()
	writeYamlConfig(t, db, "mowgli:\n  ros__parameters: {}\n")

	s, err := loadMqttBrokerSettings(db)
	require.NoError(t, err)
	assert.Equal(t, mqttBrokerSettings{
		enabled: false, host: "localhost", port: 1883, topicPrefix: "mowgli",
	}, s)
}

func TestLoadMqttBrokerSettings_MissingFileIsNotAnError(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte("/does/not/exist.yaml")))

	s, err := loadMqttBrokerSettings(db)
	require.NoError(t, err)
	assert.False(t, s.enabled)
}

// ===========================================================================
// mqtt_enabled: false means the bridge touches no network
// ===========================================================================

func TestNewScheduleMqttBridge_DisabledIsInertAndDoesNotPanic(t *testing.T) {
	db := types.NewMockDBProvider()
	writeYamlConfig(t, db, "mowgli:\n  ros__parameters:\n    mqtt_enabled: false\n")

	assert.NotPanics(t, func() {
		b := NewScheduleMqttBridge(db)
		assert.Nil(t, b.client)
	})
}
