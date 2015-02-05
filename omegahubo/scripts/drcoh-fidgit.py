#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */



import hubo_ach as ha
import ach
import sys
import time
import copy
import random as ran
from ctypes import *

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
#s.flush()
#r.flush()

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()
ref_save = ha.HUBO_REF()
ref_new = ha.HUBO_REF()

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)
[statusr, framesizer] = r.get(ref, wait=False, last=False)
ref_save = copy.deepcopy(ref)
ref_new = copy.deepcopy(ref)
n = 0
nn = 200

k = 0.2
L = 300.0

while True:
  if (n >= nn ):
    n = 0
    for i in range(0,ha.HUBO_JOINT_COUNT):
      x = ran.random()
      x = (x*2.0 - 1.0)*k
      ref_new.ref[i] = x
      if (i == ha.LEB):
        ref_new.ref[ha.LEB] = ref_new.ref[ha.LEB] - 1.1
      if (i == ha.REB):
        ref_new.ref[ha.REB] = ref_new.ref[ha.REB] - 1.1

  for i in range(0,ha.HUBO_JOINT_COUNT):
    ref.ref[i] = (ref.ref[i] * (L-1.0) + ref_new.ref[i]) / (L);


# wheels
  lw = 1.3
  rw = -1.3
  ref.ref[ha.LAP] = lw
  ref.ref[ha.RAP] = rw
  ref.ref[ha.LAR] = lw
  ref.ref[ha.RAR] = rw
  r.put(ref)
  n = n+1
  time.sleep(0.01)

# Close the connection to the channels
r.close()
s.close()

