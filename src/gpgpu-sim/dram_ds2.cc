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
#include "dram_ds2.h"
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
void dram_ds2_t::read_complete(unsigned id, uint64_t address, uint64_t clock_cycle, void *mf_return)
{
    mem_fetch *data=(mem_fetch *)mf_return;

    if (data->dec_cont()==0){ //ya ha vuelto el último Request de todos los que se generaron:
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
    ql--; //disminuimos que_length
}

void dram_ds2_t::write_complete(unsigned id, uint64_t address, uint64_t clock_cycle, void *mf_return)
{
    mem_fetch *data=(mem_fetch *)mf_return;

    if (data->dec_cont()==0){ //ya ha vuelto el último Request de todos los que se generaron:
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
    ql--; //disminuimos que_length
}

//dram_ds2_t::dram_ds2_t(){};

dram_ds2_t::dram_ds2_t( unsigned int partition_id, const struct memory_config *m_config, memory_stats_t *stats, memory_partition_unit *mp ) : dram_t( partition_id, m_config, stats, mp)

{
    ql=0; //que_length = 0;
    type = dramsim2;
    std::string dev(m_config->dramsim2_controller_ini);
    std::string sys(m_config->dramsim2_dram_ini);

    vis = new std::string(m_config->dramsim2_vis_file);   //std::string vis(m_config->dramsim2_vis_file)  (probar, y si funciona borrar la declaracion de vis del .h);
    objDramSim2 = new MultiChannelMemorySystem(dev, sys, "", "", m_config->dramsim2_total_memory_megs, vis);
    returnq = new fifo_pipeline<mem_fetch>("dramreturnq",0,m_config->gpgpu_dram_return_queue_size==0?1024:m_config->gpgpu_dram_return_queue_size);
    TransactionCompleteCB *read_cb = new Callback<dram_ds2_t, void, unsigned, uint64_t, uint64_t,void>(this, &dram_ds2_t::read_complete);
    TransactionCompleteCB *write_cb = new Callback<dram_ds2_t, void, unsigned, uint64_t, uint64_t,void>(this, &dram_ds2_t::write_complete);
    objDramSim2->RegisterCallbacks(read_cb, write_cb, NULL);
}



bool dram_ds2_t::full(new_addr_type addr) const
{
  //BUSCAR EN EL CORREO LA LLAMADA A 'FULL' DE DRAMSIM2
  bool b=not objDramSim2->willAcceptTransaction(addr);
  //printf("*** dram_t::full devuelve ") ;
  //fputs(b ? "true)\n" : "false)\n", stdout);
  //exit(0);
  return b;



  /*CODIGO original
  bool dram_t::full() const
{
    if(m_config->scheduler_type == DRAM_FRFCFS ){
        if(m_config->gpgpu_frfcfs_dram_sched_queue_size == 0 ) return false;
        return m_frfcfs_scheduler->num_pending() >= m_config->gpgpu_frfcfs_dram_sched_queue_size;
    }
   else return mrqq->full();
}

BORRAR:
bool b=backup_de_MF.size()>=5;
std::cout << "El tamaño de la cola es " << backup_de_MF.size() << '\n';
printf("*** METODO dram_t:full NO IMPLEMENTADO! (devolviendo ") ;
fputs(b ? "true)\n" : "false)\n", stdout);
//exit(0);
return b;


*/
}

unsigned dram_ds2_t::que_length() const
{
//std::cout << "\nTamaño de la cola: " << ql << '\n';
return ql;
}
  //std::cout << "El tamaño de la cola es " << backup_de_MF.size() << '\n';
  //printf("Tamaño de la cola: %u", ql);

  //return backup_de_MF.size();
  //return (unsigned) backup_de_MF.size();

//IMPRIMIR MENSAJE DE 'NO IMPLEMENTADO' Y SALIR

/* que_length ORIGINAL:
unsigned dram_t::que_length() const
{
   unsigned nreqs = 0;
   if (m_config->scheduler_type == DRAM_FRFCFS ) {
      nreqs = m_frfcfs_scheduler->num_pending();
   } else {
      nreqs = mrqq->get_length();
   }
   return nreqs;
}
*/

bool dram_ds2_t::returnq_full() const
{
   return returnq->full();
}

unsigned int dram_ds2_t::queue_limit() const
{
  //MIRAR SI SE USA
  printf("*** METODO dram_t:queue_limit SI SE USA!") ;
  exit(0);
   return m_config->gpgpu_frfcfs_dram_sched_queue_size;
}

/*
dram_req_t::dram_req_t( class mem_fetch *mf )
{
   txbytes = 0;
   dqbytes = 0;
   data = mf;

   const addrdec_t &tlx = mf->get_tlx_addr();

   bk  = tlx.bk;
   row = tlx.row;
   col = tlx.col;
   nbytes = mf->get_data_size();

   timestamp = gpu_tot_sim_cycle + gpu_sim_cycle;
   addr = mf->get_addr();
   insertion_time = (unsigned) gpu_sim_cycle;
   rw = data->get_is_write()?DRAM_WRITE:DRAM_READ;
}
*/
void dram_ds2_t::push( class mem_fetch *data )
{ 
   data->set_status(IN_PARTITION_MC_INTERFACE_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);
   int m=4; //multiplicador. La implementacion correcta ha de calcularlo de los parámetro de la arquitectura.
   int s=32; //incremento sobre la dirección base del mf
   data->set_cont(m); //este mem_fetch ha generado 4 operaciones = 4 Requests
   for (int j=0;j<m;j++){ //mandamos un lote de requests, tienen el mismo mf, pero van a direcciones distintas (contiguas)
	objDramSim2->addTransaction(data->is_write(), data->get_addr()+s*j, data);
        ql++; //aumentamos que_length
   }
   // stats...
   //estas estadísticas creo que sólo se actualizan para cada push, no para cada una de las X operaciones que genera:
   n_req += 1;
   n_req_partial += 1;
   max_mrqs_temp = (max_mrqs_temp > que_length())? max_mrqs_temp : que_length();
   m_stats->memlatstat_dram_access(data);
   
    //el metodo 'Push' nativo de la clase 'dram_t' de gpgpu-sim, se corresponde con el metodo 'Addtransaction' nativo de Dramsim2.
    //'Push' recibe como parametro un objeto 'mem_fetch' (mem_feth.(h,cc) ), mientras que 'Addtransaction' recibe un objeto 'transaction' (transaction.(h,cc))
    //No circulan datos, únicamente una dirección de memoria y tipo de operacion (lectura o escritura)
    //Los obtenemos a partir de los metodos de mem_fetch is_write() y get_addr(), y los pasamos directamente a addTransaction del DramSim2

//#define DEC2ZERO(x) x = (x)? (x-1) : 0;
//#define SWAP(a,b) a ^= b; b ^= a; a ^= b;
}
void dram_ds2_t::cycle()
{
objDramSim2->update();
}


void dram_ds2_t::print( FILE* simFile) const
{
  fprintf(simFile, "dramsim no imprime estadisticas!!\n");
}
