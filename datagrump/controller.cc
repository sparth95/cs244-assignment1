#include <iostream>
#include <cmath>
#include <limits>
#include <cstddef>
#include <vector>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ),
    alpha(2.f),
    beta(0.7f),
    window_size_(1.f),
    state(0),
    update(true),
    outstanding(0),
    rtt(-1.f),
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
      outstanding = window_size();
      window_size_ = max(1.f, window_size_ * beta);
    }
  }  
  else{
    if(state == 0){
      window_size_ = max(1.f, window_size_ + alpha/window_size_);
    } else if(state == 1 && update){
      if(outstanding == 0)
        outstanding = window_size();
      window_size_ = window_size_ * 1.25f; // probe
    } else if(state == 2 && update){
      outstanding = window_size();
      window_size_ = (window_size_ / 1.25f) * 0.75f; // counter the queues
    } else if(state == 3 && update){
      outstanding = window_size();
      window_size_ = window_size_/0.75f; // stable pace
    } else if(state == 4 && update){
      outstanding = window_size();  // stable pace
      window_size_ = window_size_;
    } else if(state == 3){
      window_size_ = max(1.f, window_size_ + alpha/window_size_);
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
    cerr << "At time " << timestamp_ms()
	 << " window size is " << the_window_size << endl;
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

  update = (outstanding == 0);

  /* AIMD: multiplicative decrease on timeout */
  if(after_timeout){
    update_member(true);
  }

  if ( debug_ && false) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << " (timeout = " << after_timeout << ")\n";
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
  // refine the set 
  while(true){
    set<pair<uint64_t, uint64_t> >::iterator it = ts_rtt.begin();
    
    int keep = 2;
    if(state > 0)
      keep = 4;

    if((timestamp_ack_received - (*it).first) > (keep*(float)rtt_)){
      ts_rtt.erase(it);
    }else{
      break;
    }
  }

  // update outstanding number of packets
  outstanding = max(0, outstanding - 1);
  update = (outstanding == 0);

  float min_rtt, mean, dev;
  get_stat(min_rtt, mean, dev);
  bool stable = (dev/mean < 0.1);
  
  // transisition to stable state
  if(state == 0 && (stable || rtt_/min_rtt < 1.1)){ // queue has cleared
    state = 1;
  } else if(state == 1 && outstanding == 0){
    state = 2;
  } else if(state == 2 && outstanding == 0){
    state = 3;
  } else if(state == 3 && outstanding == 0){
    state = 4;
  } else if(state == 4 && outstanding == 0){
    // state = 5;
  // } else if(state == 5 && outstanding == 0){
    // decision
    float min_rtt = 10000000;
    float max_rtt = 0;
    set<pair<uint64_t, uint64_t> > :: iterator it = ts_rtt.begin();
    while(it != ts_rtt.end()){
      float rtt_t = (*it).second;
      min_rtt = min(rtt_t, min_rtt);
      max_rtt = max(rtt_t, max_rtt);
      it++;
    }
    if(max_rtt/min_rtt < 1.5 || dev/mean < 0.1){
      state = 1;  // go again into probing
      outstanding = window_size();
      window_size_ = window_size_*1.25; // probe successful change baseline
    } else {
      state = 0; // go to unstable phase
    }
  }

  bool halved = false;
  float rtt_new;

  if(rtt < 0){
    rtt_new = rtt_;
    rtt = rtt_;
  }
  else {
    float alpha = 0.8;
    rtt_new = alpha*rtt + (1-alpha)*rtt_;
  }

  rtt = rtt_new;

  if(rtt_/min_rtt > 1.5 && state == 0){ // halving only when unstable
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
      << " , RTT: " << (int)rtt_
      << " , RTT_old: " << (int)rtt
      << " , RTT_new: " << (int)rtt_new
      << " , CWND: " << window_size() 
      << " , halved: " << halved
      << " , update: " << update
      << " , mean: " << (int)mean
      << " , dev: " << (int)dev
      << " , stable: " << stable
      << " , state: " << state
      << ", outstanding: " << outstanding
      << endl;
  }

}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms()
{
  return 80; /* timeout of one second */
}
