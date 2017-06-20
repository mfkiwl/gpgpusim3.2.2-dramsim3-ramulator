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
#include "dram.h"
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
void dram_t::read_complete(unsigned id, uint64_t address, uint64_t clock_cycle, void *mf_return)
{
    returnq->push((mem_fetch *) mf_return);
    ql--; //disminuimos que_length


  //recuperar de la lista  el mem_Fetch asociado a esta operacion:
 /*
  mem_fetch *mf_return;

  std::map<new_addr_type, class mem_fetch*>::iterator it;
  it=backup_de_MF.find((new_addr_type) address);
  if(it != backup_de_MF.end()){
     mf_return = it->second; //si existe tal mem_fectch en el backup, asociarlo a mf_Return
   }
     //backup_de_MF.erase((new_addr_type) address); // y borrarlo de backup
     printf("[Callback] read complete: %d 0x%lx cycle=%lu\n", id, address, clock_cycle);
     backup_de_MF.erase(it);




  //m_memory_partition_unit->get_sub_partition(mf_return->get_sub_partition_id())->dram_L2_queue_push(mf_return);
  */
}

void dram_t::write_complete(unsigned id, uint64_t address, uint64_t clock_cycle, void *mf_return)
{

  returnq->push((mem_fetch *)mf_return);
  ql--; //disminuimos que_length

  /*
  //recuperar de la lista  el mem_Fetch asociado a esta operacion.
  mem_fetch *mf_return;
  std::map<new_addr_type, class mem_fetch*>::iterator it;
  it=backup_de_MF.find((new_addr_type) address);
  if(it != backup_de_MF.end()){
    mf_return = it->second; //si existe tal mem_fectch en el backup, asociarlo a mf_Return
  }
  printf("[Callback] write complete: %d 0x%lx cycle=%lu\n", id, address, clock_cycle);
  backup_de_MF.erase((new_addr_type) address); // y borrarlo de backup


  //m_memory_partition_unit->get_sub_partition(mf_return->get_sub_partition_id())->dram_L2_queue_push(mf_return);
  */
}


dram_t::dram_t( unsigned int partition_id, const struct memory_config *config, memory_stats_t *stats,
                memory_partition_unit *mp )
{

   id = partition_id;
   m_memory_partition_unit = mp;
   m_stats = stats;
   m_config = config;

   ql=0; //que_length = 0;

/*

struct dram_system_handler_t* dram_system_create(const char *dev_desc_file, const char *sys_desc_file, unsigned int total_memory_megs, const char *vis_file)
{
std::string dev(dev_desc_file);
std::string sys(sys_desc_file);

    std::string vis(vis_file);
    return new MultiChannelMemorySystem(dev, sys, "", "", total_memory_megs, vis);
}

*/

    std::string dev(m_config->dramsim2_controller_ini);
    std::string sys(m_config->dramsim2_dram_ini);

    vis = new std::string(m_config->dramsim2_vis_file);
    //std::string vis(m_config->dramsim2_vis_file);

   //LA SIGUIENTE LINEA ESTÁ COPIADA DEL EJEMPLO DE USO DE DRAMSIM2 PARA COMPROBAR QUE ES POSIBLE LLAMARLO DESDE AQUI
   //SUSTITUIRLA POR LA INVOCACION EQUIVALENTE USANDO NUESTROS PARÁMETROS
   //objDramSim2 = new MultiChannelMemorySystem(dev, sys, "", "", tunk, &vis);
   objDramSim2 = new MultiChannelMemorySystem(dev, sys, "", "", m_config->dramsim2_total_memory_megs, vis);


    // printf("*** YUHUUUUUUUUUUU");
     //printf("%s\n",vis.c_str());

   //objDramSim2 = new MultiChannelMemorySystem("ini/DDR2_micron_16M_8b_x8_sg3E.ini", "system.ini", "..", "example_app", 16384);

   //objDramSim2 = new MultiChannelMemorySystem(dev, sys, nulo,nulo, m_config->dramsim2_total_memory_megs, vis);
   //objDramSim2 = new getMemorySystemInstance(dev, sys, nulo,nulo,(unsigned)16384, nulo);

  //  objDramSim2 = new MultiChannelMemorySystem( string(m_config->dramsim2_controller_ini),string(m_config->dramsim2_dram_ini), "", "", m_config->dramsim2_total_memory_megs,string(m_config->dramsim2_vis_file));
//  objDramSim2 = new MultiChannelMemorySystem( str1, str2,"", "",m_config->dramsim2_total_memory_megs, str3);
/*
  MultiChannelMemorySystem::MultiChannelMemorySystem(const string &deviceIniFilename_, const string &systemIniFilename_, const string &pwd_, const string &traceFilename_, unsigned megsOfMemory_, string *visFilename_, const IniReader::OverrideMap *paramOverrides)
  	:megsOfMemory(megsOfMemory_), deviceIniFilename(deviceIniFilename_),
  	systemIniFilename(systemIniFilename_), traceFilename(traceFilename_),
  	pwd(pwd_), visFilename(visFilename_),
  	clockDomainCrosser(new ClockDomain::Callback<MultiChannelMemorySystem, void>(this, &MultiChannelMemorySystem::actual_update)),
  	csvOut(new CSVWriter(visDataOut))
  {
*/

  returnq = new fifo_pipeline<mem_fetch>("dramreturnq",0,m_config->gpgpu_dram_return_queue_size==0?1024:m_config->gpgpu_dram_return_queue_size);


  TransactionCompleteCB *read_cb = new Callback<dram_t, void, unsigned, uint64_t, uint64_t,void>(this, &dram_t::read_complete);
  TransactionCompleteCB *write_cb = new Callback<dram_t, void, unsigned, uint64_t, uint64_t,void>(this, &dram_t::write_complete);

   objDramSim2->RegisterCallbacks(read_cb, write_cb, NULL);


}



bool dram_t::full(new_addr_type addr) const
{
  //BUSCAR EN EL CORREO LA LLAMADA A 'FULL' DE DRAMSIM2
  bool b=not objDramSim2->willAcceptTransaction(addr);
  printf("*** dram_t::full devuelve ") ;
  fputs(b ? "true)\n" : "false)\n", stdout);
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

unsigned dram_t::que_length() const
{

  printf("*****************\n");
  printf("*  _        _   *\n");
  printf("*  O        O   *\n");
  printf("*       |       *\n");
  printf("*  *        *   *\n");
  printf("*   ********    *\n");
  printf("*****************\n");
  printf("\n");

  //std::cout << "El tamaño de la cola es " << backup_de_MF.size() << '\n';
  printf("Tamaño de la cola: %u", ql);

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


}

bool dram_t::returnq_full() const
{
   return returnq->full();
}

unsigned int dram_t::queue_limit() const
{
  //MIRAR SI SE USA
  printf("*** METODO dram_t:queue_limit SI SE USA!") ;
  exit(0);
   return m_config->gpgpu_frfcfs_dram_sched_queue_size;
}


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

void dram_t::push( class mem_fetch *data )
{

    //el metodo 'Push' nativo de la clase 'dram_t' de gpgpu-sim, se corresponde con el metodo 'Addtransaction' nativo de Dramsim2.
    //'Push' recibe como parametro un objeto 'mem_fetch' (mem_feth.(h,cc) ), mientras que 'Addtransaction' recibe un objeto 'transaction' (transaction.(h,cc))
    //No circulan datos, únicamente una dirección de memoria y tipo de operacion (lectura o escritura)
    //Los obtenemos a partir de los metodos de mem_fetch is_write() y get_addr(), y los pasamos directamente a addTransaction del DramSim2

    //En este punto debe existir una lista en la cual poder introducir los mem_fetch recibidos, introducir el recibido y recuperarlo posteriormente.

    //DA ERROR:

    //typedef std::pair<unsigned long long, mem_fetch> Taddr_memfetchPair;

    //backup_de_MF.insert(Taddr_memfetchPair(data->get_addr(), data)); //guardamos el objeto MemFetch asociado a la dirección de memoria que solicita

    //backup_de_MF.insert(std::make_pair(data->get_addr(), data)); //guardamos el objeto MemFetch asociado a la dirección de memoria que solicita



   assert(id == data->get_tlx_addr().chip); // Ensure request is in correct memory partition
   objDramSim2->addTransaction(data->is_write(), data->get_addr(), data);
   printf("\nAñadimos acceso a la dirección %ull\n",data->get_addr());
   ql++; //aumentamos que_length

}

void dram_t::scheduler_fifo()
{
   if (!mrqq->empty()) {
      unsigned int bkn;
      dram_req_t *head_mrqq = mrqq->top();
      head_mrqq->data->set_status(IN_PARTITION_MC_BANK_ARB_QUEUE,gpu_sim_cycle+gpu_tot_sim_cycle);
      bkn = head_mrqq->bk;
      if (!bk[bkn]->mrq)
         bk[bkn]->mrq = mrqq->pop();
   }
}


#define DEC2ZERO(x) x = (x)? (x-1) : 0;
#define SWAP(a,b) a ^= b; b ^= a; a ^= b;

void dram_t::cycle()
{
/* cycle original
  if( !returnq->full() ) {
      dram_req_t *cmd = rwq->pop();
      if( cmd ) {
#ifdef DRAM_VIEWCMD
          printf("\tDQ: BK%d Row:%03x Col:%03x", cmd->bk, cmd->row, cmd->col + cmd->dqbytes);
#endif
          cmd->dqbytes += m_config->dram_atom_size;
          if (cmd->dqbytes >= cmd->nbytes) {
             mem_fetch *data = cmd->data;
             data->set_status(IN_PARTITION_MC_RETURNQ,gpu_sim_cycle+gpu_tot_sim_cycle);
             if( data->get_access_type() != L1_WRBK_ACC && data->get_access_type() != L2_WRBK_ACC ) {
                data->set_reply();
                returnq->push(data);
             } else {
                m_memory_partition_unit->set_done(data);
                delete data;
             }
             delete cmd;
          }
*/
//cycle original hasta aqui

objDramSim2->update();
}

//if mrq is being serviced by dram, gets popped after CL latency fulfilled
class mem_fetch* dram_t::return_queue_pop()
{
    return returnq->pop();
}

class mem_fetch* dram_t::return_queue_top()
{
    return returnq->top();
}

void dram_t::print( FILE* simFile) const
{
   unsigned i;
   fprintf(simFile,"DRAM[%d]: %d bks, busW=%d BL=%d CL=%d, ",
           id, m_config->nbk, m_config->busW, m_config->BL, m_config->CL );
   fprintf(simFile,"tRRD=%d tCCD=%d, tRCD=%d tRAS=%d tRP=%d tRC=%d\n",
           m_config->tCCD, m_config->tRRD, m_config->tRCD, m_config->tRAS, m_config->tRP, m_config->tRC );
   fprintf(simFile,"n_cmd=%d n_nop=%d n_act=%d n_pre=%d n_req=%d n_rd=%d n_write=%d bw_util=%.4g\n",
           n_cmd, n_nop, n_act, n_pre, n_req, n_rd, n_wr,
           (float)bwutil/n_cmd);
   fprintf(simFile,"n_activity=%d dram_eff=%.4g\n",
           n_activity, (float)bwutil/n_activity);
   for (i=0;i<m_config->nbk;i++) {
      fprintf(simFile, "bk%d: %da %di ",i,bk[i]->n_access,bk[i]->n_idle);
   }
   fprintf(simFile, "\n");
   fprintf(simFile, "dram_util_bins:");
   for (i=0;i<10;i++) fprintf(simFile, " %d", dram_util_bins[i]);
   fprintf(simFile, "\ndram_eff_bins:");
   for (i=0;i<10;i++) fprintf(simFile, " %d", dram_eff_bins[i]);
   fprintf(simFile, "\n");
   if(m_config->scheduler_type== DRAM_FRFCFS)
       fprintf(simFile, "mrqq: max=%d avg=%g\n", max_mrqs, (float)ave_mrqs/n_cmd);
}

void dram_t::visualize() const
{
   printf("RRDc=%d CCDc=%d mrqq.Length=%d rwq.Length=%d\n",
          RRDc, CCDc, mrqq->get_length(),rwq->get_length());
   for (unsigned i=0;i<m_config->nbk;i++) {
      printf("BK%d: state=%c curr_row=%03x, %2d %2d %2d %2d %p ",
             i, bk[i]->state, bk[i]->curr_row,
             bk[i]->RCDc, bk[i]->RASc,
             bk[i]->RPc, bk[i]->RCc,
             bk[i]->mrq );
      if (bk[i]->mrq)
         printf("txf: %d %d", bk[i]->mrq->nbytes, bk[i]->mrq->txbytes);
      printf("\n");
   }
   if ( m_frfcfs_scheduler )
      m_frfcfs_scheduler->print(stdout);
}

void dram_t::print_stat( FILE* simFile )
{
   fprintf(simFile,"DRAM (%d): n_cmd=%d n_nop=%d n_act=%d n_pre=%d n_req=%d n_rd=%d n_write=%d bw_util=%.4g ",
           id, n_cmd, n_nop, n_act, n_pre, n_req, n_rd, n_wr,
           (float)bwutil/n_cmd);
   fprintf(simFile, "mrqq: %d %.4g mrqsmax=%d ", max_mrqs, (float)ave_mrqs/n_cmd, max_mrqs_temp);
   fprintf(simFile, "\n");
   fprintf(simFile, "dram_util_bins:");
   for (unsigned i=0;i<10;i++) fprintf(simFile, " %d", dram_util_bins[i]);
   fprintf(simFile, "\ndram_eff_bins:");
   for (unsigned i=0;i<10;i++) fprintf(simFile, " %d", dram_eff_bins[i]);
   fprintf(simFile, "\n");
   max_mrqs_temp = 0;
}

void dram_t::visualizer_print( gzFile visualizer_file )
{
   // dram specific statistics
   gzprintf(visualizer_file,"dramncmd: %u %u\n",id, n_cmd_partial);
   gzprintf(visualizer_file,"dramnop: %u %u\n",id,n_nop_partial);
   gzprintf(visualizer_file,"dramnact: %u %u\n",id,n_act_partial);
   gzprintf(visualizer_file,"dramnpre: %u %u\n",id,n_pre_partial);
   gzprintf(visualizer_file,"dramnreq: %u %u\n",id,n_req_partial);
   gzprintf(visualizer_file,"dramavemrqs: %u %u\n",id,
            n_cmd_partial?(ave_mrqs_partial/n_cmd_partial ):0);

   // utilization and efficiency
   gzprintf(visualizer_file,"dramutil: %u %u\n",
            id,n_cmd_partial?100*bwutil_partial/n_cmd_partial:0);
   gzprintf(visualizer_file,"drameff: %u %u\n",
            id,n_activity_partial?100*bwutil_partial/n_activity_partial:0);

   // reset for next interval
   bwutil_partial = 0;
   n_activity_partial = 0;
   ave_mrqs_partial = 0;
   n_cmd_partial = 0;
   n_nop_partial = 0;
   n_act_partial = 0;
   n_pre_partial = 0;
   n_req_partial = 0;

   // dram access type classification
   for (unsigned j = 0; j < m_config->nbk; j++) {
      gzprintf(visualizer_file,"dramglobal_acc_r: %u %u %u\n", id, j,
               m_stats->mem_access_type_stats[GLOBAL_ACC_R][id][j]);
      gzprintf(visualizer_file,"dramglobal_acc_w: %u %u %u\n", id, j,
               m_stats->mem_access_type_stats[GLOBAL_ACC_W][id][j]);
      gzprintf(visualizer_file,"dramlocal_acc_r: %u %u %u\n", id, j,
               m_stats->mem_access_type_stats[LOCAL_ACC_R][id][j]);
      gzprintf(visualizer_file,"dramlocal_acc_w: %u %u %u\n", id, j,
               m_stats->mem_access_type_stats[LOCAL_ACC_W][id][j]);
      gzprintf(visualizer_file,"dramconst_acc_r: %u %u %u\n", id, j,
               m_stats->mem_access_type_stats[CONST_ACC_R][id][j]);
      gzprintf(visualizer_file,"dramtexture_acc_r: %u %u %u\n", id, j,
               m_stats->mem_access_type_stats[TEXTURE_ACC_R][id][j]);
   }
}


void dram_t::set_dram_power_stats(	unsigned &cmd,
									unsigned &activity,
									unsigned &nop,
									unsigned &act,
									unsigned &pre,
									unsigned &rd,
									unsigned &wr,
									unsigned &req) const{

	// Point power performance counters to low-level DRAM counters
	cmd = n_cmd;
	activity = n_activity;
	nop = n_nop;
	act = n_act;
	pre = n_pre;
	rd = n_rd;
	wr = n_wr;
	req = n_req;
}
