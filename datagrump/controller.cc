#include <iostream>
#include <cmath>

#include "controller.hh"
#include "timestamp.hh"

using namespace std;

/* Default constructor */
Controller::Controller( const bool debug )
  : debug_( debug ),
    alpha(2.f),
    beta(0.5f),
    window_size_(1.f)
{}

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
  /* AIMD: multiplicative decrease on timeout */
  if(after_timeout)
    window_size_ = max(1.f, window_size_ * beta);

  if ( debug_ ) {
    cerr << "At time " << send_timestamp
	 << " sent datagram " << sequence_number << " (timeout = " << after_timeout << ")\n";
  }

  window_size();
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
  if ( debug_ ) {
    cerr << "At time " << timestamp_ack_received
	 << " received ack for datagram " << sequence_number_acked
	 << " (send @ time " << (int)send_timestamp_acked
	 << ", received @ time " << (int)recv_timestamp_acked << " by receiver's clock" 
   << ", difference " << ((int)timestamp_ack_received - (int)send_timestamp_acked) << ")"
	 << endl;
  }

  if((timestamp_ack_received - send_timestamp_acked) > timeout_ms()){
    window_size_ = max(1.f, window_size_ * beta);
  } else {
    /* AIMD: additive increase */
    window_size_ = max(1.f, window_size_ + alpha/window_size_);
  }
  window_size();
}

/* How long to wait (in milliseconds) if there are no acks
   before sending one more datagram */
unsigned int Controller::timeout_ms()
{
  return 150; /* timeout of one second */
}
