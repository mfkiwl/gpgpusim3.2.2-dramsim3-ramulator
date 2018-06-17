//
//1. Hacer una almacen de mem_fetchs, y rellenarla en los 'push' y recuperarlos-vaciarla en los metodos 'callback'
//2. Hacer una fifo llamada returnq (contiene los mf recuperados en el paso anterior), se llena en el callback (en lugar de la dram_to_L2) y se vacia en cycle antes de realizar una operacion sobre dramsim2
//3.




// Copyright (c) 2009-2011, Tor M. Aamodt, Wilson W.L. Fung, Ali Bakhoda,
// Ivan Sham, George L. Yuan,
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

#include "map"
#include "gpu-sim.h"
#include "gpu-misc.h"
#include "dram_ramulator.h"
#include "mem_latency_stat.h"
#include "dram_sched.h"
#include "l2cache.h"

//#include "../DRAMSim2/MultiChannelMemorySystem.h"

//#include "../DRAMSim2/DRAMSim.h"
#include <string>
#include <stdint.h>
#include <stdio.h>

#ifdef DRAM_VERIFY
int PRINT_CYCLE = 0;
#endif

template class fifo_pipeline<mem_fetch>;
template class fifo_pipeline<dram_req_t>;

//typedef std::pair<unsigned long long, class mem_fetch*> Taddr_memfetchPair;

/* callback functors */
/*
void dram_ramulator_t::read_complete(ramulator::Request& req)
{
    mem_fetch *data=(mem_fetch *)req.mf;
    data->set_status(IN_PARTITION_MC_RETURNQ,gpu_sim_cycle+gpu_tot_sim_cycle);

    ql--; //disminuimos que_length
    data->set_reply();
    returnq->push(data);
    cont--;
    //std::cout  << data->get_request_uid() << " Sale  --- es un read. id: " << id << " Pendientes: "<< cont << "\n";
}

vo
*/
void dram_ramulator_t::readwrite_complete(ramulator::Request& req)
{
  mem_fetch *data=(mem_fetch *)req.mf;
  //std::cout  << data->get_request_uid() << " Sale  --- es un write. id: " << id << " Pendientes: "<< cont << "\n";
  ql--; //disminuimos que_length
  data->set_status(IN_PARTITION_MC_RETURNQ,gpu_sim_cycle+gpu_tot_sim_cycle);

  if( data->get_access_type() != L1_WRBK_ACC && data->get_access_type() != L2_WRBK_ACC ) {
     data->set_reply();
     returnq->push(data);
  } else {
     m_memory_partition_unit->set_done(data);
     delete data;
  }

}

//dram_ds2_t::dram_ds2_t(){};

dram_ramulator_t::dram_ramulator_t( unsigned int partition_id, const struct memory_config *m_config, memory_stats_t *stats, memory_partition_unit *mp ) : dram_t( partition_id, m_config, stats, mp)

{
    ql=0; //que_length = 0;
    type = dramulator;
    rw_cb_func=std::bind(&dram_ramulator_t::readwrite_complete, this, std::placeholders::_1);
    //write_cb_func=std::bind(&dram_ramulator_t::readwrite_complete, this, std::placeholders::_1);

    std::string cfg(m_config->ramulator_config_file);
    objRamulator = new ramulator::Gem5Wrapper(cfg, m_config->m_L2_config.get_line_sz());
}
/*
dram_ramulator_t::~dram_ramulator_t() {
  printf("*** se finaliza objRamulator!") ;
  objRamulator->finish();
  //delete objRamulator;
}
*/
bool dram_ramulator_t::full() const
{
  if (ql<2) return false; else return true;
}

bool dram_ramulator_t::full(mem_fetch* mf) const
{
  //std::cout << "llamada a ramulator.full() " << ql << '\n';
  bool b;
  ramulator::Request req;
  if (mf->is_write())
    ramulator::Request req(mf->get_addr(), ramulator::Request::Type::WRITE, (int) id);
  else
    ramulator::Request req(mf->get_addr(), ramulator::Request::Type::READ, (int) id);
  b=objRamulator->full(req);
  if (b) std::cout << "ramulator.full() = true -- tam_cola=" << ql << '\n';
  return (b);
}

unsigned dram_ramulator_t::que_length() const
{
  //std::cout << "Tamaño de la cola: " << ql << '\n';
  return ql;
}

/*
unsigned int dram_ramulator_t::queue_limit() const
{
  //MIRAR SI SE USA
  printf("*** METODO dram_t:queue_limit SI SE USA!") ;
  exit(0);
   return m_config->gpgpu_frfcfs_dram_sched_queue_size;
}
*/

void dram_ramulator_t::push( class mem_fetch *data )
{
   assert(id == data->get_tlx_addr().chip); // Ensure request is in correct memory partition
   data->set_status(IN_PARTITION_DRAM,gpu_sim_cycle+gpu_tot_sim_cycle);

   ramulator::Request *req;

   if (data->is_write()){
     req = new ramulator::Request(data->get_addr(), ramulator::Request::Type::WRITE, this->rw_cb_func, data, (int)  id);
     //std::cout  << data->get_request_uid() << " Entra --- es un write. id: " << id << " Pendientes: "<< cont << "\n";
   }else{
     req = new ramulator::Request(data->get_addr(), ramulator::Request::Type::READ, this->rw_cb_func, data,  (int) id);
     //std::cout  << data->get_request_uid() << " Entra --- es un read.  id: " << id << " Pendientes: "<< cont << "\n";
   }

   if (!objRamulator->send(*req)) std::cout  << " Error en el SEND de ramulator.\n";
   ql++; //aumentamos que_length
}

void dram_ramulator_t::cycle()
{
  //std::cout  << " Ciclo de GPGPUSIM \n";
    objRamulator->tick();
}

void dram_ramulator_t::print( FILE* simFile) const
{
  fprintf(simFile, "ramulator no imprime estadisticas!!\n");
}
