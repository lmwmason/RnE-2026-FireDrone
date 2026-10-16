/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "platform.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "research/firedrone.h"
PG_REGISTER_WITH_RESET_TEMPLATE(firedroneConfig_t, firedroneConfig, PG_FIREDRONE_CONFIG, 0);
PG_RESET_TEMPLATE(firedroneConfig_t, firedroneConfig,
    .mode=0, .blend=0, .ukf=0, .axis=0, .auxChannel=0,
    .effectiveness={0,0,0}, .bandwidth=20, .rateGain=15,
    .motorTauMs=25, .derivativeHz=30, .correctionLimit=25, .adaptation=0
);
