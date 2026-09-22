package api

import (
	"encoding/json"
	"log"
	"sync"
	"time"

	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/types"
)

const teleopLeaseDuration = 3 * time.Second

type teleopRequest struct {
	Type    string                `json:"type"`
	Command geometry.TwistStamped `json:"command"`
}

type teleopState struct {
	Type     string `json:"type"`
	State    string `json:"state"`
	Revision uint64 `json:"revision"`
}

type teleopClient struct {
	conn    *websocket.Conn
	writeMu sync.Mutex
}

func (c *teleopClient) writeState(state teleopState) error {
	c.writeMu.Lock()
	defer c.writeMu.Unlock()
	_ = c.conn.SetWriteDeadline(time.Now().Add(wsWriteTimeout))
	return c.conn.WriteJSON(state)
}

// teleopController arbitrates the single /cmd_vel_teleop publisher shared by
// every browser. A connection must own the lease before velocity commands are
// accepted. Global stop deliberately has no ownership check.
type teleopController struct {
	mu       sync.Mutex
	provider types.IRosProvider
	clients  map[*teleopClient]struct{}
	owner    *teleopClient
	blocked  bool
	timer    *time.Timer
	lease    time.Duration
	revision uint64
}

func newTeleopController(provider types.IRosProvider) *teleopController {
	return &teleopController{
		provider: provider,
		clients:  make(map[*teleopClient]struct{}),
		lease:    teleopLeaseDuration,
	}
}

func (t *teleopController) register(client *teleopClient) {
	t.mu.Lock()
	t.clients[client] = struct{}{}
	state := t.stateForLocked(client)
	t.mu.Unlock()
	if err := client.writeState(state); err != nil {
		_ = client.conn.Close()
	}
}

func (t *teleopController) unregister(client *teleopClient) {
	t.mu.Lock()
	delete(t.clients, client)
	wasOwner := t.owner == client
	if wasOwner {
		t.clearOwnerLocked()
		t.revision++
	}
	t.mu.Unlock()
	if wasOwner {
		t.publishZero()
		t.broadcast()
	}
}

func (t *teleopController) acquire(client *teleopClient) {
	t.mu.Lock()
	if t.blocked || (t.owner != nil && t.owner != client) {
		state := t.stateForLocked(client)
		t.mu.Unlock()
		if err := client.writeState(state); err != nil {
			_ = client.conn.Close()
		}
		return
	}
	changed := t.owner != client
	t.owner = client
	t.refreshLeaseLocked(client)
	if changed {
		t.revision++
	}
	t.mu.Unlock()
	t.broadcast()
}

// enable starts a new explicit teleop session after a previous global stop.
// A release, disconnect, or lease expiry does not block takeover; STOP does.
func (t *teleopController) enable() {
	t.mu.Lock()
	if !t.blocked {
		t.mu.Unlock()
		return
	}
	t.blocked = false
	t.revision++
	t.mu.Unlock()
	t.broadcast()
}

func (t *teleopController) heartbeat(client *teleopClient) {
	t.mu.Lock()
	t.refreshLeaseLocked(client)
	t.mu.Unlock()
}

func (t *teleopController) release(client *teleopClient) {
	t.mu.Lock()
	if t.owner != client {
		t.mu.Unlock()
		return
	}
	t.clearOwnerLocked()
	t.revision++
	t.mu.Unlock()
	t.publishZero()
	t.broadcast()
}

func (t *teleopController) command(client *teleopClient, command geometry.TwistStamped) {
	t.mu.Lock()
	if t.owner != client {
		t.mu.Unlock()
		return
	}
	t.refreshLeaseLocked(client)
	t.mu.Unlock()
	if err := t.provider.Publish("/cmd_vel_teleop", "geometry_msgs/msg/TwistStamped", &command); err != nil {
		log.Printf("teleopController: publish error: %v", err)
	}
}

// globalStop is intentionally callable without a lease. It immediately sends
// zero velocity and revokes the current owner, so stale commands from that
// connection cannot restart motion without a new acquire.
func (t *teleopController) globalStop() {
	t.mu.Lock()
	t.clearOwnerLocked()
	t.blocked = true
	t.revision++
	t.mu.Unlock()
	t.publishZero()
	t.broadcast()
}

func (t *teleopController) refreshLeaseLocked(client *teleopClient) {
	if t.owner != client {
		return
	}
	if t.timer != nil {
		t.timer.Stop()
	}
	t.timer = time.AfterFunc(t.lease, func() {
		t.expire(client)
	})
}

func (t *teleopController) expire(client *teleopClient) {
	t.mu.Lock()
	if t.owner != client {
		t.mu.Unlock()
		return
	}
	t.clearOwnerLocked()
	t.revision++
	t.mu.Unlock()
	t.publishZero()
	t.broadcast()
}

func (t *teleopController) clearOwnerLocked() {
	if t.timer != nil {
		t.timer.Stop()
		t.timer = nil
	}
	t.owner = nil
}

func (t *teleopController) stateForLocked(client *teleopClient) teleopState {
	state := "available"
	if t.blocked {
		state = "stopped"
	} else if t.owner == client {
		state = "owner"
	} else if t.owner != nil {
		state = "busy"
	}
	return teleopState{Type: "teleop_state", State: state, Revision: t.revision}
}

func (t *teleopController) broadcast() {
	t.mu.Lock()
	states := make(map[*teleopClient]teleopState, len(t.clients))
	for client := range t.clients {
		states[client] = t.stateForLocked(client)
	}
	t.mu.Unlock()
	for client, state := range states {
		go func() {
			if err := client.writeState(state); err != nil {
				_ = client.conn.Close()
			}
		}()
	}
}

func (t *teleopController) publishZero() {
	zero := geometry.TwistStamped{}
	if err := t.provider.Publish("/cmd_vel_teleop", "geometry_msgs/msg/TwistStamped", &zero); err != nil {
		log.Printf("teleopController: stop publish error: %v", err)
	}
}

func (t *teleopController) handle(client *teleopClient, payload []byte) {
	var request teleopRequest
	if err := json.Unmarshal(payload, &request); err != nil {
		log.Printf("teleopController: unmarshal error: %v", err)
		return
	}
	switch request.Type {
	case "acquire":
		t.acquire(client)
	case "heartbeat":
		t.heartbeat(client)
	case "release":
		t.release(client)
	case "command":
		t.command(client, request.Command)
	case "stop":
		t.globalStop()
	}
}
