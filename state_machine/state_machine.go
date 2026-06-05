package salad

import (
	"fmt"
	"sync"
	"context"
	"time"
	"go.viam.com/rdk/logging"
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

type StateMachine struct {
	mu           sync.RWMutex
	status       Status
	progress     float64
	customerName string
	errorMsg     string
	opCancelFunc func()
	opDone       chan struct{}
	logger 		 logging.Logger
	buildID      string
}

func NewStateMachine() (*StateMachine){
	sm := &StateMachine {
		status: Idle,
	}
	return sm
}

func (sm *StateMachine) UpdateStateMachineStatus(status Status, progress float64) {
	sm.mu.Lock()
	defer sm.mu.Unlock()
	sm.status = status
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

func (sm *StateMachine) GetBuildID() string {
	sm.mu.RLock()
	defer sm.mu.RUnlock()
	return sm.buildID
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

	sm.logger.Infof("Stop requested, cancelling operation")
	cancelFunc()
	<-done

	return map[string]interface{}{
		"success": true,
		"message": "Operation stopped",
	}, nil
}

func (sm *StateMachine) StartBuildSalad(ctx context.Context, cancelCtx context.Context, value interface{}, customerName string) (map[string]interface{}, context.Context, error) {
	
	// Verify we're transitioning from a valid state.
	if (!(sm.status == Idle || sm.status == Complete)) {
		return nil, nil, fmt.Errorf("Salad must be in idle or complete state")
		// TODO check if these two states are right
	}
	
	sm.mu.Lock()
	if sm.opCancelFunc != nil {
		sm.mu.Unlock()
		return map[string]interface{}{
			"success": false,
			"message": "An operation is already in progress, use 'stop' to cancel it first",
		}, nil, nil
	}
	buildCtx, buildCancelFunc := context.WithCancel(cancelCtx)
	sm.opCancelFunc = buildCancelFunc
	sm.opDone = make(chan struct{})
	sm.status = "preparing"
	sm.progress = 0
	sm.customerName = customerName
	sm.errorMsg = ""
	sm.buildID = fmt.Sprintf("%s_%s", time.Now().UTC().Format("20060102-150405"), uuid.NewString()[:8])
	sm.mu.Unlock()
	sm.logger.Infof("Build ID: %s", sm.buildID)
	/////
	return nil, buildCtx, nil
}

func (sm *StateMachine) EndBuildSalad() {
	sm.mu.Lock()
	sm.opCancelFunc = nil
	sm.customerName = ""
	sm.buildID = ""
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