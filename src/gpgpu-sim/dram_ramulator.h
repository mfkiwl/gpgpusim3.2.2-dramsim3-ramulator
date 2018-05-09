// Copyright (c) 2009-2011, Tor M. Aamodt, Ivan Sham, Ali Bakhoda,
// George L. Yuan, Wilson W.L. Fung
// The University of British Columbia
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// Redistributions in binary form must reproduce the above copyright notice, this
// list of conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.
// Neither the name of The University of British Columbia nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef DRAM_RAMULATOR_H
#define DRAM_RAMULATOR_H

#include "delayqueue.h"
#include <set>
#include <zlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "../abstract_hardware_model.h"

#include "mem_fetch.h"

#include "dram.h"

#include "../ramulator-master/src/Request.h"
#include "../ramulator-master/src/Gem5Wrapper.h"

struct mem_fetch;

class dram_ramulator_t : public dram_t
{
public:

   dram_ramulator_t( unsigned int partition_id, const struct memory_config *config, class memory_stats_t *stats,
           class memory_partition_unit *mp );


   ramulator::Gem5Wrapper *objRamulator;
   std::string  *cfg;
   unsigned ql; //para llevar la cuenta de que_length al usar ramulator

   std::function<void(ramulator::Request&)> read_cb_func;
   std::function<void(ramulator::Request&)> write_cb_func;

   /* callback functions */
   void read_complete(ramulator::Request& req);
   void write_complete(ramulator::Request& req);


   bool full(new_addr_type addr, enum mem_access_type tipo) const;

   unsigned que_length() const;

   bool returnq_full() const;

   unsigned int queue_limit() const;

   void push( class mem_fetch *data );

   void cycle();

   void print( FILE* simFile) const;

};

#endif /*DRAM_H*/
