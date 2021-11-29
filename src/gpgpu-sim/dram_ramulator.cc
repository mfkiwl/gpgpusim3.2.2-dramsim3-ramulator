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

void dram_ramulator_t::read_complete(ramulator::Request& req)
{
    //std::cout << "RAMULATOR: Read Complete\n ";
    mem_fetch *data=(mem_fetch *)req.mf;
    //std::cout  << "<-R." << data->get_request_uid() << "#:" << id << " Pendientes: "<< que_length() << "\n";
    //printf("R_mf_rtt: %llu\n",gpu_sim_cycle+gpu_tot_sim_cycle-(data->get_m_status_change()));
    data->set_status(IN_PARTITION_MC_RETURNQ,gpu_sim_cycle+gpu_tot_sim_cycle);
    if( data->get_access_type() != L1_WRBK_ACC && data->get_access_type() != L2_WRBK_ACC ) {
          data->set_reply();
          returnq->push(data);
        //  std::cout << "Era un read: No es L1_WRBK_ACC ni  L2_WRBK_ACC \n ";
    } else {
          m_memory_partition_unit->set_done(data);
          delete data;
        //  std::cout << "Era un read: Es L1_WRBK_ACC o  L2_WRBK_ACC \n ";
      }
}

void dram_ramulator_t::write_complete(ramulator::Request& req)
{
  //std::cout << "RAMULATOR: Write Complete\n ";
  //std::cout << "Era un WRITE \n ";
  mem_fetch *data=(mem_fetch *)req.mf;
  //printf("W_mf_rtt: %llu\n",gpu_sim_cycle+gpu_tot_sim_cycle-(data->get_m_status_change()));
  //std::cout  << "<-W." << data->get_request_uid() << "#:" << id << " Pendientes: "<< que_length() << "\n";
//ql--; //disminuimos que_length
  data->set_status(IN_PARTITION_MC_RETURNQ,gpu_sim_cycle+gpu_tot_sim_cycle);
  //std::cout << " Sale  id: "  << data->get_request_uid() << " Tipo: "<< data->get_access_type() << "\n";
  if( data->get_access_type() != L1_WRBK_ACC && data->get_access_type() != L2_WRBK_ACC ) {
        data->set_reply();
        returnq->push(data);
        //std::cout << "Era un write: No es L1_WRBK_ACC ni  L2_WRBK_ACC \n ";
  } else {
        m_memory_partition_unit->set_done(data);
        delete data;
        //std::cout << "Era un write: Es L1_WRBK_ACC o  L2_WRBK_ACC \n ";
    }
}

//dram_ds2_t::dram_ds2_t(){};

dram_ramulator_t::dram_ramulator_t( unsigned int partition_id, const struct memory_config *config, memory_stats_t *stats, memory_partition_unit *mp ) : dram_t( partition_id, config, stats, mp)

{
  /*
  id = partition_id;
  m_memory_partition_unit = mp;
  m_stats = stats;
  m_config = config;

  CCDc = 0;
  RRDc = 0;
  RTWc = 0;
  WTRc = 0;

  rw = DRAM_READ; //read mode is default

 bkgrp = (bankgrp_t**) calloc(sizeof(bankgrp_t*), m_config->nbkgrp);
 bkgrp[0] = (bankgrp_t*) calloc(sizeof(bank_t), m_config->nbkgrp);
 for (unsigned i=1; i<m_config->nbkgrp; i++) {
   bkgrp[i] = bkgrp[0] + i;
 }
 for (unsigned i=0; i<m_config->nbkgrp; i++) {
   bkgrp[i]->CCDLc = 0;
   bkgrp[i]->RTPLc = 0;
 }

  bk = (bank_t**) calloc(sizeof(bank_t*),m_config->nbk);
  bk[0] = (bank_t*) calloc(sizeof(bank_t),m_config->nbk);
  for (unsigned i=1;i<m_config->nbk;i++)
     bk[i] = bk[0] + i;
  for (unsigned i=0;i<m_config->nbk;i++) {
     bk[i]->state = BANK_IDLE;
     bk[i]->bkgrpindex = i/(m_config->nbk/m_config->nbkgrp);
  }
  prio = 0;
  rwq = new fifo_pipeline<dram_req_t>("rwq",m_config->CL,m_config->CL+1);
  mrqq = new fifo_pipeline<dram_req_t>("mrqq",0,2);
  returnq = new fifo_pipeline<mem_fetch>("dramreturnq",0,m_config->gpgpu_dram_return_queue_size==0?1024:m_config->gpgpu_dram_return_queue_size);
  m_frfcfs_scheduler = NULL;
  if ( m_config->scheduler_type == DRAM_FRFCFS )
     m_frfcfs_scheduler = new frfcfs_scheduler(m_config,this,stats);
  n_cmd = 0;
  n_activity = 0;
  n_nop = 0;
  n_act = 0;
  n_pre = 0;
  n_rd = 0;
  n_wr = 0;
  n_req = 0;
  max_mrqs_temp = 0;
  bwutil = 0;
  max_mrqs = 0;
  ave_mrqs = 0;

  for (unsigned i=0;i<10;i++) {
     dram_util_bins[i]=0;
     dram_eff_bins[i]=0;
  }
  last_n_cmd = last_n_activity = last_bwutil = 0;

  n_cmd_partial = 0;
  n_activity_partial = 0;
  n_nop_partial = 0;
  n_act_partial = 0;
  n_pre_partial = 0;
  n_req_partial = 0;
  ave_mrqs_partial = 0;
  bwutil_partial = 0;

  */
     mrqq_Dist = StatCreate("mrqq_length",1,64); //track up to 64 entries

    ql=1; //que_length = 0;
    type = dramulator;
    read_cb_func=std::bind(&dram_ramulator_t::read_complete, this, std::placeholders::_1);
    write_cb_func=std::bind(&dram_ramulator_t::write_complete, this, std::placeholders::_1);

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
  //std::cout << "llamada a ramulator.full() " << '\n';
  //por compatibilidad con gpgpusim, devolvemos false
  //eso permite continuar la ejecucion hasta el momento justo de encolar la petición
  //en cuyo caso ya podemos llamar a full(mem_fetch*) y obtener el resultado
  return false;
}


bool dram_ramulator_t::full(mem_fetch* mf) const
{
  //std::cout << "llamada a dram_ramulator.full(*mf) " << ql << '\n';
  bool b;
  ramulator::Request *req;
  /* asi no va
  if (mf->get_type()==0)//READ_REQUEST=0
    ramulator::Request req(mf->get_addr(), ramulator::Request::Type::READ, (int) id);
  else
    ramulator::Request req(mf->get_addr(), ramulator::Request::Type::WRITE, (int) id);
    */

  if (mf->get_type()==0){//READ_REQUEST=0
    req = new ramulator::Request(mf->get_addr(), ramulator::Request::Type::READ, this->read_cb_func, mf, (int)  id);
   //std::cout  << "->R." << data->get_request_uid() << "#:" << id << " Pendientes: "<< que_length() << "\n";
  }else{
    req = new ramulator::Request(mf->get_addr(), ramulator::Request::Type::WRITE, this->write_cb_func, mf,  (int) id);
         //std::cout  << "->W." << data->get_request_uid() << "#:" << id << " Pendientes: "<< que_length() << "\n";
  }


  b=objRamulator->full(*req);

  return (b);
}

unsigned dram_ramulator_t::que_length() const
{
  //std::cout << "Tamaño de la cola: " << ql << '\n';
  return objRamulator->que_length();
}

/*
unsigned int dram_ramulator_t::queue_limit() const
{
  return 64
}
*/

void dram_ramulator_t::push( class mem_fetch *data )
{
   //assert(id == data->get_tlx_addr().chip); // Ensure request is in correct memory partition
   id=0;//Ramulator que se apañe
   
   data->set_status(IN_PARTITION_MC_INTERFACE_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);

   ramulator::Request *req;

   //borrar:
   //std::cout << "Atom size:" << m_config->dram_atom_size << "\n"; 

   if (data->get_type()==0){//READ_REQUEST=0
     req = new ramulator::Request(data->get_addr(), ramulator::Request::Type::READ, this->read_cb_func, data, (int)  id);
     //std::cout  << "->R." << data->get_request_uid() << "#:" << id << " Pendientes: "<< que_length() << "\n";
   }else{
     req = new ramulator::Request(data->get_addr(), ramulator::Request::Type::WRITE, this->write_cb_func, data,  (int) id);
     //std::cout  << "->W." << data->get_request_uid() << "#:" << id << " Pendientes: "<< que_length() << "\n";
   }
   // stats...
      n_req += 1;
      n_req_partial += 1;
      max_mrqs_temp = (max_mrqs_temp > objRamulator->que_length())? max_mrqs_temp : objRamulator->que_length();
      m_stats->memlatstat_dram_access(data);

   if (!objRamulator->send(*req)){
      std::cout  << " Error en el PUSH de ramulator.\n";
      assert(false);
    }
   //ql++; //aumentamos que_length
}

void dram_ramulator_t::cycle()
{
  //std::cout  << " Ciclo de GPGPUSIM \n";
    //for (ql=0;ql<16;ql++)  objRamulator->tick();
    //objRamulator->tick();
    objRamulator->tick();
}

void dram_ramulator_t::print( FILE* simFile) const
{
  fprintf(simFile, "ramulator no imprime estadisticas!!\n");
}
