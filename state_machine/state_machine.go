package salad

import (
	"fmt"
	"sync"
	"context"
)

type Status string

// As our system's complexity increases, inlining mutextes will without any guardrails will inevitbly lead to deadlocks.
// status shouldn't be strings - should be a constant enum type.
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
	opCancelFunc func()
	opDone       chan struct{}
}

func NewStateMachine() (*StateMachine){
	sm := &StateMachine {
		status: Idle,
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

func (sm *StateMachine) GetStateMachineStatus() map[string]interface{} {
	sm.mu.RLock()
	defer sm.mu.RUnlock()
	return map[string]interface{}{
		"status":        sm.status,
		"progress":      sm.progress,
		"customer_name": sm.customerName,
		"error_msg":     sm.errorMsg,
	}
}

func (sm *StateMachine) DoStop() (map[string]interface{}, error) {
	sm.mu.RLock()
	cancelFunc := sm.opCancelFunc
	done := sm.opDone
	sm.mu.RUnlock()

	if cancelFunc == nil {
		return map[string]interface{}{
			"success": false,
			"message": "No operation in progress",
		}, nil
	}

	// TODO add logger for state machine
	// s.logger.Infof("Stop requested, cancelling operation")
	cancelFunc()
	<-done

	return map[string]interface{}{
		"success": true,
		"message": "Operation stopped",
	}, nil
}

func (sm *StateMachine) StartBuildSalad(ctx context.Context, cancelCtx context.Context, value interface{}, customerName string) (map[string]interface{}, context.Context) {
	
	// TODO: Verify we're transitioning from a valid state.
	
	sm.mu.Lock()
	if sm.opCancelFunc != nil {
		sm.mu.Unlock()
		return map[string]interface{}{
			"success": false,
			"message": "An operation is already in progress, use 'stop' to cancel it first",
		}, nil
	}
	buildCtx, buildCancelFunc := context.WithCancel(cancelCtx)
	sm.opCancelFunc = buildCancelFunc
	sm.opDone = make(chan struct{})
	sm.status = "preparing"
	sm.progress = 0
	sm.customerName = customerName
	sm.errorMsg = ""
	sm.mu.Unlock()
	/////
	return nil, buildCtx
}

func (sm *StateMachine) EndBuildSalad() {
	sm.mu.Lock()
	sm.opCancelFunc = nil
	sm.customerName = ""
	close(sm.opDone)
	sm.opDone = nil
	sm.mu.Unlock()
}

func (sm *StateMachine) BuildSaladFailed(failMsg string) () {
	sm.mu.Lock()
	sm.status = "failed"
	sm.errorMsg = failMsg
	sm.mu.Unlock()
}

func (sm *StateMachine) OperationInProgress(cancelCtx context.Context) (map[string]interface{}, context.Context) {
	sm.mu.Lock()
	if sm.opCancelFunc != nil {
		sm.mu.Unlock()
		return map[string]interface{}{
			"success": false,
			"message": "An operation is already in progress, use 'stop' to cancel it first",
		}, nil
	}
	setupCtx, setupCancelFunc := context.WithCancel(cancelCtx)
	sm.opCancelFunc = setupCancelFunc
	sm.opDone = make(chan struct{})
	sm.status = "setting_up_station"
	sm.progress = 0
	sm.errorMsg = ""
	sm.mu.Unlock()
	return nil, setupCtx
}

func (sm *StateMachine) EndSetupStation() {
	sm.mu.Lock()
	sm.opCancelFunc = nil
	close(sm.opDone)
	sm.opDone = nil
	sm.mu.Unlock()
}

func (sm *StateMachine) SetupStationError(err error) (map[string]interface{}) {
	sm.mu.Lock()
	sm.status = "failed"
	sm.errorMsg = err.Error()
	sm.mu.Unlock()
	return map[string]interface{}{
		"success": false,
		"message": err.Error(),
	}
}