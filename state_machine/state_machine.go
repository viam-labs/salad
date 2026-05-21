package salad

import (
	"sync"
)

type StateMachine struct {
	mu           sync.RWMutex
	progress     float64
	customerName string
	errorMsg     string
	simulate     bool
	opCancelFunc func()
	opDone       chan struct{}
}

type Status string

// todo finish this, find correct values
// need to ask how to add ingredients into the SM - should we just have
// one state for adding ingredients or divide them up?
const (
	Idle       Status = "idle"
	Preparing  Status = "preparing"
	Stopped    Status = "stopped"
	Failed     Status = "failed"
	SettingUp  Status = "setting_up_station"
	Adding     Status = "adding_ingredient"
	Delivering Status = "delivering_salad"
	Complete   Status = "complete"
)

// TODO  initialize variables

// TODO change status to the enum
func (s *StateMachine) updateStatus(status string, progress float64) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.status = status
	s.progress = progress
}
