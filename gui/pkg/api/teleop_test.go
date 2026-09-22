package api

import (
	"encoding/json"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func teleopTestServer(t *testing.T, lease time.Duration) (*httptest.Server, *types.MockRosProvider, *teleopController) {
	t.Helper()
	gin.SetMode(gin.TestMode)
	provider := types.NewMockRosProvider()
	controller := newTeleopController(provider)
	controller.lease = lease
	router := gin.New()
	PublisherRoute(router.Group("/api/mowglinext"), controller)
	return httptest.NewServer(router), provider, controller
}

func connectTeleop(t *testing.T, server *httptest.Server) *websocket.Conn {
	t.Helper()
	url := "ws" + strings.TrimPrefix(server.URL, "http") + "/api/mowglinext/publish/joy"
	conn, _, err := websocket.DefaultDialer.Dial(url, nil)
	require.NoError(t, err)
	t.Cleanup(func() { _ = conn.Close() })
	assertTeleopState(t, conn, "available")
	return conn
}

func assertTeleopState(t *testing.T, conn *websocket.Conn, want string) teleopState {
	t.Helper()
	require.NoError(t, conn.SetReadDeadline(time.Now().Add(time.Second)))
	_, payload, err := conn.ReadMessage()
	require.NoError(t, err)
	var state teleopState
	require.NoError(t, json.Unmarshal(payload, &state))
	assert.Equal(t, "teleop_state", state.Type)
	assert.Equal(t, want, state.State)
	return state
}

func sendTeleop(t *testing.T, conn *websocket.Conn, request any) {
	t.Helper()
	require.NoError(t, conn.WriteJSON(request))
}

func twist(linear float64) geometry.TwistStamped {
	command := geometry.TwistStamped{}
	command.Twist.Linear.X = linear
	return command
}

func TestTeleopOnlyOwnerCanPublishAndAnyClientCanStop(t *testing.T) {
	server, provider, controller := teleopTestServer(t, time.Second)
	defer server.Close()
	owner := connectTeleop(t, server)
	observer := connectTeleop(t, server)

	sendTeleop(t, owner, map[string]any{"type": "acquire"})
	assertTeleopState(t, owner, "owner")
	assertTeleopState(t, observer, "busy")

	// A non-owner's joystick frame, including its ordinary zero frame, is ignored.
	sendTeleop(t, observer, map[string]any{"type": "command", "command": twist(0)})
	sendTeleop(t, owner, map[string]any{"type": "command", "command": twist(0.2)})
	require.Eventually(t, func() bool {
		return len(provider.GetPublishes()) == 1
	}, time.Second, 10*time.Millisecond)
	published := provider.GetPublishes()
	require.IsType(t, &geometry.TwistStamped{}, published[0].Msg)
	assert.Equal(t, 0.2, published[0].Msg.(*geometry.TwistStamped).Twist.Linear.X)

	// Explicit stop is globally accepted and revokes the old owner's lease.
	sendTeleop(t, observer, map[string]any{"type": "stop"})
	assertTeleopState(t, owner, "stopped")
	assertTeleopState(t, observer, "stopped")
	published = provider.GetPublishes()
	require.Len(t, published, 2)
	assert.Equal(t, float64(0), published[1].Msg.(*geometry.TwistStamped).Twist.Linear.X)

	// The former owner cannot resume on stale frames or immediately reacquire.
	sendTeleop(t, owner, map[string]any{"type": "command", "command": twist(0.3)})
	sendTeleop(t, observer, map[string]any{"type": "acquire"})
	assertTeleopState(t, observer, "stopped")
	require.Len(t, provider.GetPublishes(), 2)

	// A new explicit manual/recording session unlocks acquisition.
	controller.enable()
	assertTeleopState(t, owner, "available")
	assertTeleopState(t, observer, "available")
	sendTeleop(t, observer, map[string]any{"type": "acquire"})
	assertTeleopState(t, owner, "busy")
	assertTeleopState(t, observer, "owner")
	sendTeleop(t, observer, map[string]any{"type": "command", "command": twist(-0.1)})
	require.Eventually(t, func() bool {
		return len(provider.GetPublishes()) == 3
	}, time.Second, 10*time.Millisecond)
	assert.Equal(t, -0.1, provider.GetPublishes()[2].Msg.(*geometry.TwistStamped).Twist.Linear.X)
}

func TestTeleopLeaseExpiryStopsAndReleasesControl(t *testing.T) {
	server, provider, _ := teleopTestServer(t, 50*time.Millisecond)
	defer server.Close()
	owner := connectTeleop(t, server)
	observer := connectTeleop(t, server)

	sendTeleop(t, owner, map[string]any{"type": "acquire"})
	assertTeleopState(t, owner, "owner")
	assertTeleopState(t, observer, "busy")
	assertTeleopState(t, observer, "available")
	assertTeleopState(t, owner, "available")

	require.Len(t, provider.GetPublishes(), 1)
	assert.Equal(t, float64(0), provider.GetPublishes()[0].Msg.(*geometry.TwistStamped).Twist.Linear.X)
}

func TestTeleopOwnerDisconnectStopsAndReleasesControl(t *testing.T) {
	server, provider, _ := teleopTestServer(t, time.Second)
	defer server.Close()
	owner := connectTeleop(t, server)
	observer := connectTeleop(t, server)

	sendTeleop(t, owner, map[string]any{"type": "acquire"})
	assertTeleopState(t, owner, "owner")
	assertTeleopState(t, observer, "busy")
	require.NoError(t, owner.Close())

	assertTeleopState(t, observer, "available")
	require.Len(t, provider.GetPublishes(), 1)
	assert.Equal(t, float64(0), provider.GetPublishes()[0].Msg.(*geometry.TwistStamped).Twist.Linear.X)
}
