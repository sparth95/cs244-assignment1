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
    alpha(2.f),
    beta(0.7f),
    window_size_(1.f),
    state(0),
    update(true),
    outstanding(0),
    prob_probability(base_prob_probability),
    rtt(0),
    queue_delay(0),
    q_(0),
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

void Controller::update_member(bool timeout, int state = 0)
{

  if(timeout){
    if(update){
      if(outstanding == 0)
        outstanding = window_size();
      window_size_ = max(1.f, window_size_ * beta);
    }
  }  
  else{
    if(state == 0){
      // window_size_ = max(1.f, window_size_ + alpha/window_size_);
      window_size_ = window_size_; // mo increase in unstable
    } else if(state == 1){
      if(outstanding == 0)
        outstanding = window_size();
      if(step_inc < 0)
        window_size_ = max(1.f, window_size_ + alpha/window_size_);
        // window_size_ = window_size_;
      else 
        window_size_ = max(1.f, window_size_ + step_inc);
    } else if(state == 2){
      if(outstanding == 0)
        outstanding = window_size();
      
      window_size_ = max(1.f, window_size_ + step_inc);
      // window_size_ = window_size_*inc; // probe
    } else if(state == 3 && update){
      if(outstanding == 0)
        outstanding = window_size();
      window_size_ = (window_size_/inc)* (2-inc); // control queue
    }
  }

  window_size_ = max(1.f, window_size_);
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

  // update = (outstanding == 0);
// 
  /* AIMD: multiplicative decrease on timeout */
  if(after_timeout){
    update_member(true);
  }

  if ( debug_) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << " queue_delay " << queue_delay << " (timeout = " << after_timeout << ")\n";
  }
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
  uint64_t rtt_ = (timestamp_ack_received - send_timestamp_acked);
  ts_rtt.insert(make_pair(timestamp_ack_received, rtt_));
  
  // update_queue_delay
  pair<long, long> current_ack = make_pair(send_timestamp_acked, recv_timestamp_acked);
  q_ = max((long)0, q_ + (current_ack.second - last_ack.second) - (current_ack.first - last_ack.first)); // queue delay for current packet
  queue_delay = queue_delay * 0.9f + 0.1f * q_;
  last_ack = current_ack;

  // refine the set 
  while(true){
    set<pair<uint64_t, uint64_t> >::iterator it = ts_rtt.begin();
    
    int keep = 4;

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

  float min_rtt, mean, dev;
  get_stat(min_rtt, mean, dev);

  // bool stable = (dev/mean < 0.1) || (min(rtt_, (timestamp_ack_received - send_timestamp_acked))/min_rtt < 1.1);
  // bool panic = (timestamp_ack_received - send_timestamp_acked)/min_rtt > 2;

  bool stable = (q_ < 10) || (rtt_/min_rtt < 1.1);
  bool panic = (q_/max(queue_delay, 0.000001f) > 3) || (rtt_/min_rtt > 2); // max for numeric stability
  
  // bool queue_cleared = ((timestamp_ack_received - send_timestamp_acked)/min_rtt) < 1.1;
  // state transitions
  // 0 : unstable
  // 1 : stable
  // 2 : prob
  // 3 : cool off
  if(panic && state != 0 && state != 3){ // significant increase in the rtt compared to prvious min
    
    // shortcut to unstable
    if(step_inc > 0){
      window_size_ += outstanding*step_inc; // the current window size
    }

    if(state == 1){
      window_size_ = window_size_ /(inc);
    }

    prob_probability = base_prob_probability;
    state = 0;
    outstanding = 0;
    cerr << "short circuit to state 0" << window_size_ << endl;
  }
  else if(state == 0 && stable){ // queue has cleared
    state = 1;
    outstanding = 0;
    prob_probability = base_prob_probability;
  } else if(state == 1 && outstanding == 0){
    if(stable){
      float seed = (rand() %100 )/ 100.0;
      if(seed < prob_probability){
        state = 2;
        float new_window = window_size_ * inc;
        int steps = window_size(); // current_outstanding packets
        step_inc = (new_window - window_size_)/steps;
      }else{
        state = 1;
        step_inc = -1.f;
        prob_probability = min(1.0, prob_probability*1.1); 
      }
    } else {
      state = 0;
    }
  } else if(state == 2 && outstanding == 0){
    state = 3;
  } else if(state == 3 && outstanding == 0){
    if(stable){
      outstanding = window_size();

      int steps = outstanding;
      float new_window = (window_size_/(2-inc))*inc*inc; // probe successful change baseline and being next probe
      step_inc = (new_window - window_size_)/steps;
      state = 2;
    } else {
      outstanding = window_size();
      
      int steps = outstanding;
      float new_window = (window_size_/(2-inc));
      step_inc = (new_window - window_size_)/steps;

      prob_probability = base_prob_probability;
      state = 1; // go to stable phase
    }
  }

  // reset step_inc when not needed
  if(state == 0 || state == 3){
    step_inc = -1.f;
  }

  bool halved = false;

  if(!stable && state == 0){ // halving only when unstable
    update_member(true);
    if(update)
      halved = true;
    update = false;
  } else {
    update_member(false, state);
  }
  



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
      << " , mean: " << (int)mean
      << " , dev: " << (int)dev
      << " , stable: " << stable
      << " , state: " << state
      << ", outstanding: " << outstanding
      << ", prob: " << prob_probability
      << endl;
  }

}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms()
{
  return 2*max(rtt, 40.f); /* timeout of one second */
}
