#include "hold_trim.hpp"
#include <cassert>
#include <limits>
#include <iostream>
int main() {
  HoldTrim::Q sent{}, q{}, v{}, lo{}, hi{}; lo.fill(-2); hi.fill(2);
  sent[8]=.05; std::string error;
  HoldTrim c;
  assert(c.start(8,.05,sent,q,lo,hi,error));
  assert(c.output == sent);
  // Simple static-load plant: measured angle = output - constant load/Kp.
  for (int n=0;n<6000 && c.active();++n) {
    auto before=c.output;
    c.step(q,v,.01,true);
    assert(std::abs(c.output[8]-before[8]) <= .002*.01+1e-12);
    for(int i=0;i<16;++i) if(i!=8) assert(c.output[i]==sent[i]);
    q[8]=c.output[8]-.05;
  }
  assert(c.phase=="ready" && std::abs(q[8]-.05)<=.003);
  auto held=c.output; c.step(q,v,.01,true); assert(c.output==held);
  q.fill(0); assert(c.start(8,.05,sent,q,lo,hi,error));
  c.step(q,v,.01,false); assert(c.phase=="stopped" && c.output==sent);
  assert(c.start(8,.05,sent,q,lo,hi,error));
  v[0]=.1; c.step(q,v,.01,true); assert(c.output==sent && !c.active()); v.fill(0);
  assert(c.start(8,.05,sent,q,lo,hi,error));
  q[0]=.02; c.step(q,v,.01,true); assert(c.reason=="unselected_joint_moved"); q.fill(0);
  assert(!c.start(7,0,sent,q,lo,hi,error));
  assert(!c.start(8,.5,sent,q,lo,hi,error));
  assert(!c.start(8,std::numeric_limits<double>::quiet_NaN(),sent,q,lo,hi,error));
  // Blocked joint cannot accumulate unbounded output.
  assert(c.start(8,.05,sent,q,lo,hi,error));
  for(int n=0;n<7000 && c.active();++n) c.step(q,v,.01,true);
  assert(!c.active() && c.phase!="ready" && std::abs(c.output[8]-sent[8])<=.08);
  assert(c.start(8,.05,sent,q,lo,hi,error));
  c.step(q,v,.051,true); assert(c.reason=="feedback_or_loop_invalid" && c.output==sent);
  assert(c.start(8,.05,sent,q,lo,hi,error));
  c.freeze("operator_stop"); assert(c.output==sent && !c.active());
  std::cout << "hold_trim: load convergence, continuity, slew, isolation, stale, limits, timeout and stop passed\n";
}
