package providers

import (
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/foxglove"
	"github.com/stretchr/testify/require"
)

func TestRosProviderRetainsAndReplaysCachedStateAfterLastUnsubscribe(t *testing.T) {
	r := &RosProvider{
		client:             foxglove.NewClient("ws://unused"),
		subscribers:        make(map[string]map[string]*RosSubscriber),
		lastMessage:        make(map[string][]byte),
		foxgloveSubscribed: map[string]bool{"path": true},
	}

	first := make(chan []byte, 1)
	require.NoError(t, r.Subscribe("path", "first", 0, func(msg []byte) {
		first <- append([]byte(nil), msg...)
	}))

	want := []byte(`{"poses":[{"pose":{"position":{"x":1}}}]}`)
	r.fanOut("path", want)
	require.Equal(t, want, receiveMessage(t, first))

	r.UnSubscribe("path", "first")
	require.Equal(t, want, r.lastMessage["path"], "last state must survive the final unsubscribe")

	replayed := make(chan []byte, 1)
	require.NoError(t, r.Subscribe("path", "second", 0, func(msg []byte) {
		replayed <- append([]byte(nil), msg...)
	}))
	t.Cleanup(func() { r.UnSubscribe("path", "second") })

	require.Equal(t, want, receiveMessage(t, replayed), "new subscriber must receive cached state immediately")
}

func receiveMessage(t *testing.T, messages <-chan []byte) []byte {
	t.Helper()
	select {
	case msg := <-messages:
		return msg
	case <-time.After(2 * time.Second):
		t.Fatal("timed out waiting for cached message")
		return nil
	}
}
