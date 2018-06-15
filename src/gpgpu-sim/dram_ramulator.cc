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

//void dram_ramulator_t::read_complete(unsigned id, uint64_t address, uint64_t clock_cycle, void *mf_return)
{

    mem_fetch *data=(mem_fetch *)req.mf;
    //std::cout << "Sale el memfetch por el read_complete #" << data->get_addr() << "es un write? =" << data->is_write() << "\n";


    ql--; //disminuimos que_length

    data->set_reply();
    data->set_status(IN_PARTITION_MC_RETURNQ,gpu_sim_cycle+gpu_tot_sim_cycle);
    returnq->push(data);
    cont--;
    std::cout  << data->get_request_uid() << " Sale  --- es un read. id: " << id << " Pendientes: "<< cont << "\n";

}

void dram_ramulator_t::write_complete(ramulator::Request& req)
{
  mem_fetch *data=(mem_fetch *)req.mf;
  //std::cout << "Sale el memfetch por el write_complete #" << data->get_addr() << "es un write? =" << data->is_write() << "\n";
  cont--;
  std::cout  << data->get_request_uid() << " Sale  --- es un write. id: " << id << " Pendientes: "<< cont << "\n";


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
    read_cb_func=std::bind(&dram_ramulator_t::read_complete, this, std::placeholders::_1);
    write_cb_func=std::bind(&dram_ramulator_t::write_complete, this, std::placeholders::_1);
//AQUI HAY QUE SABER QUE FICHEROS CONTIENEN LA CONFIGURACION DE RAMULATOR PARA PASARSELO
//AL CONSTRUCTOR

//AQUI HAY QUE: INCLUIR UN CAMPO EN EL FICHERO CONFIGURACION GPGPU-SIM QUE RECOJA EL FICHERO DE CONFIGURACION DE RAMULATOR
//CREAR EL CONSTRUCTOR Gem5Wrapper QUE ACEPTE FICHERO, CON EL CONSTRUIR UN OBJETO 'CONFIG' Y LLAMAR AL CONSTRUCTOR NORMAL.
    std::string cfg(m_config->ramulator_config_file);
    objRamulator = new ramulator::Gem5Wrapper(cfg, m_config->m_L2_config.get_line_sz()); //EL SIGUIENTE BLOQUE INICIALIZA UN OBJETO DRAMSIM2 Y CONFIGURA LOS CALLBACKS

//ESCRIBIR LA EQUIVALENCIA CON RAMULATOR
/*
    objDramSim2 = new MultiChannelMemorySystem(dev, sys, "", "", m_config->dramsim2_total_memory_megs, vis);
    returnq = new fifo_pipeline<mem_fetch>("dramreturnq",0,m_config->gpgpu_dram_return_queue_size==0?1024:m_config->gpgpu_dram_return_queue_size);
    TransactionCompleteCB *read_cb = new Callback<dram_ds2_t, void, unsigned, uint64_t, uint64_t,void>(this, &dram_ds2_t::read_complete);
    TransactionCompleteCB *write_cb = new Callback<dram_ds2_t, void, unsigned, uint64_t, uint64_t,void>(this, &dram_ds2_t::write_complete);
    objDramSim2->RegisterCallbacks(read_cb, write_cb, NULL);
*/
}
/*
dram_ramulator_t::~dram_ramulator_t() {
  printf("*** se finaliza objRamulator!") ;
  objRamulator->finish();
  //delete objRamulator;
}
*/

//bool dram_ramulator_t::full(new_addr_type addr, enum mem_access_type tipo) const
bool dram_ramulator_t::full(mem_fetch* mf) const
{
  ramulator::Request req;
  if (mf->is_write())
    ramulator::Request req(mf->get_addr(), ramulator::Request::Type::WRITE, (int) id);
  else
    ramulator::Request req(mf->get_addr(), ramulator::Request::Type::READ, (int) id);

  return (objRamulator->full(req));
}


unsigned dram_ramulator_t::que_length() const
{
  std::cout << "Tamaño de la cola: " << ql << '\n';
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
  //pongo el status por si  el error de deadlock es por eso
   data->set_status(IN_PARTITION_DRAM,gpu_sim_cycle+gpu_tot_sim_cycle);
   //esto igual hay que borrarlo:
   cont++;
   ramulator::Request *req;
   ql++; //aumentamos que_length
   if (data->is_write()){
     req = new ramulator::Request(data->get_addr(), ramulator::Request::Type::WRITE, this->write_cb_func, data, (int)  id);
     std::cout  << data->get_request_uid() << " Entra --- es un write. id: " << id << " Pendientes: "<< cont << "\n";
   }else{
     req = new ramulator::Request(data->get_addr(), ramulator::Request::Type::READ, this->read_cb_func, data,  (int) id);
     std::cout  << data->get_request_uid() << " Entra --- es un read.  id: " << id << " Pendientes: "<< cont << "\n";
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
