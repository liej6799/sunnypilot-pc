#include "system/camerad/cameras/camera_common.h"

#include <cassert>
#include <cstdio>

#include "common/params.h"
#include "common/util.h"

int main(int argc, char *argv[]) {
  // doesn't need RT priority since we're using isolcpus
  int ret = util::set_core_affinity({6});
  // [op9] the Ubuntu chroot's cpuset can forbid pinning to core 6 (ret != 0); RT
  // priority isn't needed here, so warn instead of aborting (upstream asserted this).
  if (ret != 0) fprintf(stderr, "[op9] set_core_affinity({6}) failed (ret=%d) -- continuing without core pinning\n", ret);

  camerad_thread();
  return 0;
}
