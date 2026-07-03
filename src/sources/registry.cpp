#include "registry.h"

namespace sources {

static BmsSource   s_bms;
static ShuntSource s_shunt;
static MpptSource  s_mppt;
static Aggregator  s_agg;

void init_registry(const Config& cfg) {
  s_shunt.init(cfg);
  s_mppt.init(cfg);
  s_agg.init(&s_bms, &s_shunt, &s_mppt);
  s_agg.set_shunt_mode(cfg.shunt_current_mode);
}

BmsSource*   bms_source()  { return &s_bms;   }
ShuntSource* shunt_source() { return &s_shunt; }
MpptSource*  mppt_source()  { return &s_mppt;  }
Aggregator*  aggregator()   { return &s_agg;   }

}  // namespace sources
