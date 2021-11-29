#include <map>

#include "Gem5Wrapper.h"
#include "Config.h"
#include "Request.h"
#include "MemoryFactory.h"
#include "Memory.h"
#include "DDR3.h"
#include "DDR4.h"
#include "LPDDR3.h"
#include "LPDDR4.h"
#include "GDDR5.h"
#include "WideIO.h"
#include "WideIO2.h"
#include "HBM.h"
#include "SALP.h"
#include "PCM.h"
#include "STTMRAM.h"

using namespace ramulator;

static map<string, function<MemoryBase *(const Config&, int)> > name_to_func = {
    {"DDR3", &MemoryFactory<DDR3>::create}, {"DDR4", &MemoryFactory<DDR4>::create},
    {"LPDDR3", &MemoryFactory<LPDDR3>::create}, {"LPDDR4", &MemoryFactory<LPDDR4>::create},
    {"GDDR5", &MemoryFactory<GDDR5>::create},
    {"WideIO", &MemoryFactory<WideIO>::create}, {"WideIO2", &MemoryFactory<WideIO2>::create},
    {"HBM", &MemoryFactory<HBM>::create},
    {"SALP-1", &MemoryFactory<SALP>::create}, {"SALP-2", &MemoryFactory<SALP>::create}, {"SALP-MASA", &MemoryFactory<SALP>::create},
    //Añadidos constructores para: DSARP, PCM, STTMRAM:
    {"DSARP", &MemoryFactory<DSARP>::create},
    {"PCM", &MemoryFactory<PCM>::create},
    {"STTMRAM", &MemoryFactory<STTMRAM>::create},
};


Gem5Wrapper::Gem5Wrapper(const Config& configs, int cacheline)
{
    const string& std_name = configs["standard"];
    assert(name_to_func.find(std_name) != name_to_func.end() && "unrecognized standard name");
    mem = name_to_func[std_name](configs, cacheline);
    tCK = mem->clk_ns();
}

Gem5Wrapper::Gem5Wrapper(std::string& config_file, int cacheline)
{
    Config configs(config_file);
  //Config configs("/home/carbaior/gpgpu-sim_distribution/src/ramulator-master/configs/GDDR5-config.cfg");
    const string& std_name = configs["standard"];
    assert(name_to_func.find(std_name) != name_to_func.end() && "unrecognized standard name");
    mem = name_to_func[std_name](configs, cacheline);
    tCK = mem->clk_ns();
}



Gem5Wrapper::~Gem5Wrapper() {
    delete mem;
}

void Gem5Wrapper::tick()
{
    mem->tick();
}

bool Gem5Wrapper::send(Request req)
{
    return mem->send(req);
}

void Gem5Wrapper::finish(void) {
    mem->finish();
}

bool Gem5Wrapper::full(Request req)
{
  //std::cout << "llamada a Gem5Wrapper.full(Request req) " << '\n';
  bool r;
  r = mem->full(req);
  /*
  if (r){
    printf ("\n * COLA ENTRADA DE RAMULATOR LLENA * \n");
    //assert(false);
  }
  */
  return r;
}

unsigned Gem5Wrapper::que_length(){
  return mem->pending_requests();
}
