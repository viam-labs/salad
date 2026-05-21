package salad

import (
	"fmt"
	"sync"
)

type Status string

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

// TODO: instead of doing this, it might make more sense to export the 
// Status enum and use it directly in updateStatus build coordinator as a parameter? 
// (this would require changing func signatures)
// I believe updateStatus is a private func so should be ok
func stringToStatus (statusString string) (Status, error) {
	switch statusString {
	case "idle":
		return Idle, nil
	case "preparing":
		return Preparing, nil
	case "stopped":
		return Stopped, nil
	case "failed":
		return Failed, nil
	case "setting_up_station":
		return SettingUp, nil
	case "adding_ingredient":
		return Adding, nil
	case "delivering_salad":
		return Delivering, nil
	case "complete":
		return Complete, nil
	default:
		return Idle, fmt.Errorf("Invalid status")
	}
}

type StateMachine struct {
	mu           sync.RWMutex
	status       Status
	progress     float64
	customerName string
	errorMsg     string
	simulate     bool
	opCancelFunc func()
	opDone       chan struct{}
}

func NewStateMachine(simulate bool) (*StateMachine){
	sm := &StateMachine {
		status: Idle,
		simulate: simulate,
	}
	return sm
}

func (sm *StateMachine) UpdateStateMachineStatus(status string, progress float64) {
	sm.mu.Lock()
	defer sm.mu.Unlock()
	statusEnum, err := stringToStatus(status) 
	sm.status = statusEnum
	if err != nil {
		// TODO return error instead? would require changing func signature
		fmt.Print("Invalid status")
	}
	sm.progress = progress
}

func (sm *StateMachine) GetStateMachineStatus(customerName string, errorMsg string) map[string]interface{} {
	sm.mu.RLock()
	defer sm.mu.RUnlock()
	return map[string]interface{}{
		"status":        sm.status,
		"progress":      sm.progress,
		"customer_name": customerName,
		"error_msg":     errorMsg,
	}
}
