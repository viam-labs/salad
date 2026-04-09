package salad

import (
	"sync"
	"time"

	"github.com/google/uuid"
)

// Order represents a customer salad order in the queue.
type Order struct {
	ID           string         `json:"id"`
	CustomerName string         `json:"customer_name"`
	Ingredients  map[string]int `json:"ingredients"`
	Status       string         `json:"status"` // queued | building | complete | cancelled | failed
	EnqueuedAt   time.Time      `json:"enqueued_at"`
}

// OrderQueue is a thread-safe FIFO order queue.
type OrderQueue struct {
	mu      sync.Mutex
	orders  []Order
	notify  chan struct{} // buffered(1), poked on enqueue to wake consumer
}

// NewOrderQueue creates a new empty order queue.
func NewOrderQueue() *OrderQueue {
	return &OrderQueue{
		notify: make(chan struct{}, 1),
	}
}

// NewOrder creates an Order with a generated UUID and current timestamp.
func NewOrder(customerName string, ingredients map[string]int) Order {
	return Order{
		ID:           uuid.New().String(),
		CustomerName: customerName,
		Ingredients:  ingredients,
		Status:       "queued",
		EnqueuedAt:   time.Now(),
	}
}

// Enqueue adds an order to the back of the queue and returns its 1-based position.
func (q *OrderQueue) Enqueue(order Order) int {
	q.mu.Lock()
	q.orders = append(q.orders, order)
	pos := len(q.orders)
	q.mu.Unlock()

	// Non-blocking poke to wake consumer.
	select {
	case q.notify <- struct{}{}:
	default:
	}

	return pos
}

// Peek returns the front order without removing it.
func (q *OrderQueue) Peek() (Order, bool) {
	q.mu.Lock()
	defer q.mu.Unlock()
	if len(q.orders) == 0 {
		return Order{}, false
	}
	return q.orders[0], true
}

// Dequeue removes and returns the front order.
func (q *OrderQueue) Dequeue() (Order, bool) {
	q.mu.Lock()
	defer q.mu.Unlock()
	if len(q.orders) == 0 {
		return Order{}, false
	}
	order := q.orders[0]
	q.orders = q.orders[1:]
	return order, true
}

// CancelByID removes an order by ID if its status is "queued".
// Returns true if the order was found and cancelled.
func (q *OrderQueue) CancelByID(id string) bool {
	q.mu.Lock()
	defer q.mu.Unlock()
	for i, o := range q.orders {
		if o.ID == id && o.Status == "queued" {
			q.orders = append(q.orders[:i], q.orders[i+1:]...)
			return true
		}
	}
	return false
}

// List returns a copy of all orders in the queue.
func (q *OrderQueue) List() []Order {
	q.mu.Lock()
	defer q.mu.Unlock()
	out := make([]Order, len(q.orders))
	copy(out, q.orders)
	return out
}

// Clear removes all orders from the queue and returns how many were removed.
func (q *OrderQueue) Clear() int {
	q.mu.Lock()
	defer q.mu.Unlock()
	n := len(q.orders)
	q.orders = nil
	return n
}

// Len returns the number of orders in the queue.
func (q *OrderQueue) Len() int {
	q.mu.Lock()
	defer q.mu.Unlock()
	return len(q.orders)
}
