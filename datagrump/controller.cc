#include <iostream>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <limits>
#include <cstddef>
#include <vector>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

float inc = 1.25f;
const float base_prob_probability = 0.4f;
float step_inc = -1.f;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ),
    alpha(3.f),
    beta(0.7f),
    window_size_(0.f),
    state(0),
    update(true),
    outstanding(0),
    prob_probability(base_prob_probability),
    rtt(-1),
    queue_delay(0),
    q_(0),
    timeout(80),
    target(1),
    left(1),
    next_transmission(0),
    last_ack(make_pair(0, 0)),
    ts_rtt(set<pair<uint64_t, uint64_t> >())
{}

void Controller::get_stat(float &min_rtt, float &mean, float &dev)
{
  int count = ts_rtt.size();
  float rtt_sum = 0.000001;
  float diff = 0.000001;
  set<pair<uint64_t, uint64_t> >::iterator it = ts_rtt.begin();
  min_rtt = 100000000000;
  while(it != ts_rtt.end()){
    float rtt = (*it).second;
    if(min_rtt == 0){
      min_rtt = rtt; 
    }
    else{
      min_rtt = min(min_rtt, rtt); 
    }
    rtt_sum += (*it).second;
    it++;
  }

  mean = ((float)rtt_sum)/((float)count);

  it = ts_rtt.begin();
  while(it != ts_rtt.end()){
    float diff_ = ((float)((*it).second) - mean);
    diff += diff_*diff_;
    it++;
  }

  dev = sqrt(diff/count); 
}

/* Get current window size, in datagrams */
unsigned int Controller::window_size()
{
  /* Default: fixed window size of 100 outstanding datagrams */

  int the_window_size = floor(window_size_);

  if ( debug_ ) {
    // cerr << "At time " << timestamp_ms()
	 // << " window size is " << the_window_size << endl;
  }

  return the_window_size;
}

/* A datagram was sent */
void Controller::datagram_was_sent( const uint64_t sequence_number,
				    /* of the sent datagram */
				    const uint64_t send_timestamp,
                                    /* in milliseconds */
				    const bool after_timeout
				    /* datagram was sent because of a timeout */ )
{

  /* AIMD: multiplicative decrease on timeout */
  if(after_timeout and left == 0){
    outstanding = target; // current outstanding
    target *= beta; // halving the target
    timeout *= 2; // exponential backoff
  }

  if(left > 0)
    left--;
  
  if(left == 0){
    timeout = 2*max((long)rtt, (long)40);
  } else {
    timeout = rtt / target; // evenly spaced;
  }

  // next transmission time
  next_transmission = send_timestamp + timeout;

  if ( debug_) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << " queue_delay " << queue_delay << " (timeout = " << after_timeout << ") " << timeout << " " << next_transmission << "\n";
  }

  timeout = max((long)1, timeout);
  window_size_ = max(1.f, window_size_ + 1); 
}

/* An ack was received */
void Controller::ack_received( const uint64_t sequence_number_acked,
			       /* what sequence number was acknowledged */
			       const uint64_t send_timestamp_acked,
			       /* when the acknowledged datagram was sent (sender's clock) */
			       const uint64_t recv_timestamp_acked,
			       /* when the acknowledged datagram was received (receiver's clock)*/
			       const uint64_t timestamp_ack_received )
                               /* when the ack was received (by sender) */
{
  window_size_ = window_size_ - 1;
  uint64_t rtt_ = (timestamp_ack_received - send_timestamp_acked);
  ts_rtt.insert(make_pair(timestamp_ack_received, rtt_));
  
  // update_queue_delay
  pair<long, long> current_ack = make_pair(send_timestamp_acked, recv_timestamp_acked);
  q_ = max((long)0, q_ + (current_ack.second - last_ack.second) - (current_ack.first - last_ack.first)); // queue delay for current packet
  if(queue_delay > 0)
    queue_delay = queue_delay * 0.9f + 0.1f * q_;
  else 
    queue_delay = q_;
  last_ack = current_ack;

  long delay = (current_ack.second - last_ack.second) - (current_ack.first - last_ack.first);

  // refine the set 
  while(true){
    set<pair<uint64_t, uint64_t> >::iterator it = ts_rtt.begin();
    
    int keep = 2;

    if((timestamp_ack_received - (*it).first) > (keep*(float)rtt_)){
      ts_rtt.erase(it);
    }else{
      break;
    }
  }

  // update outstanding number of packets
  outstanding = max(0, outstanding - 1);
  update = (outstanding == 0);

  float rtt_new;

  if(rtt < 0){
    rtt_new = rtt_;
    rtt = rtt_;
  }
  else {
    float alpha = 0.95; // moving mean
    rtt_new = alpha*rtt + (1-alpha)*rtt_;
  }

  rtt = rtt_new;
  rtt_ = rtt; // this is to remove the effect noise 

  float min_rtt, mean, dev;
  get_stat(min_rtt, mean, dev);

  // bool stable = (dev/mean < 0.1) || (min(rtt_, (timestamp_ack_received - send_timestamp_acked))/min_rtt < 1.1);
  // bool panic = (timestamp_ack_received - send_timestamp_acked)/min_rtt > 2;

  bool stable = (q_ < 10);
  bool panic = (q_/max(queue_delay, 0.000001f) > 3); // max for numeric stability
  
  // bool queue_cleared = ((timestamp_ack_received - send_timestamp_acked)/min_rtt) < 1.1;
  // state transitions
  // 0 : unstable
  // 1 : stable
  // 2 : prob
  // 3 : cool off
  bool state_change = false;
  if(panic && state != 0 && state != 3){ // significant increase in the rtt compared to prvious min
    state_change = true;
    if(state == 2){
      target = target/(inc);
    }
    prob_probability = base_prob_probability;
    state = 0;
    outstanding = 0;
    cerr << timestamp_ack_received << " short circuit to state 0 " << window_size_ << endl;
  }
  else if(state == 0 && stable){ // queue has cleared
    state_change = true;
    state = 1;
    outstanding = target;
    prob_probability = base_prob_probability;

  } else if(state == 0 && outstanding == 0){
    state_change = true;
    outstanding = target;
    left = target;
    // timeout = rtt/target + q_;
    // next_transmission = (long)timestamp_ack_received + (int)q_ + timeout;
  }else if(state == 1 && outstanding == 0){
    state_change = true;
    if(stable){
      float seed = (rand() %100 )/ 100.0;
      if(seed < prob_probability){
        state = 2;
        outstanding = target;
        target = target * inc;
      }else{
        state = 1;
        outstanding = target;
        target = target + alpha; // stable increase
        left = target;
        prob_probability = min(1.0, prob_probability*1.1); 
      }
    } else {
      outstanding = target;
      state = 0;
    }
  } else if(state == 2 && outstanding == 0){
    state_change = true;
    outstanding = target;
    target = max((long)1, (long)(target/(inc)));
    state = 3;
  } else if(state == 3 && outstanding == 0){
    state_change = true;
    if(stable){
      outstanding = target;
      target = (target/(2-inc))*inc*inc; // probe successful change baseline and being next probe
      state = 2;
    } else {
      outstanding = target;
      target = target/(2-inc);
      prob_probability = base_prob_probability;
      state = 1; // go to stable phase
    }
  }

  bool halved = false;

  if(!stable && state == 0){ // halving only when unstable
    if(update){
      state_change = true;
      outstanding = target;
      target = max((long)1, (long)(target*beta));
      halved = true;
    }
  }
  
  if(state_change){
    timeout = rtt/target + delay;
    left = target;
    next_transmission = timestamp_ack_received + timeout;
  } else {
    long time_left = next_transmission - (long)timestamp_ack_received;
    time_left += delay; // incorporate delay information
    timeout = time_left;
    next_transmission = timestamp_ack_received + timeout;
  }

  timeout = max(timeout, (long)1);
  if ( debug_ ) {
    if(false){
      cerr << "At time " << timestamp_ack_received
       << " received ack for datagram " << sequence_number_acked
       << " (send @ time " << (int)send_timestamp_acked
       << ", received @ time " << (int)recv_timestamp_acked << " by receiver's clock) " 
       << endl;
    }

    cerr << "At time " << timestamp_ack_received
      << " , RTT: " << (timestamp_ack_received - send_timestamp_acked)
      << " , CWND: " << window_size() 
      << " , packet_no: " << sequence_number_acked
      << " , q_delay: " << queue_delay 
      << " , q_: " << q_ 
      << " , halved: " << halved
      << " , update: " << update
      << " , stable: " << stable
      << " , state: " << state
      << ", outstanding: " << outstanding
      << ", prob: " << prob_probability
      << ", rtt: " << rtt
      << ", target: " << target
      << ", left: " << left 
      << ", timeout: " << timeout 
      << ", next_transmission: " << next_transmission 
      << endl;
  }
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms()
{
  return timeout; /* timeout of one second */
}
