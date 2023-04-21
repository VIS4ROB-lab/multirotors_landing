#!/usr/bin/env python3

ACTION_SET_SIZE = 10       # forward, backward, left, right, diagonal (x4), stop, descend

QUAD_FULL_STATE_SIZE = 10
STATE_SIZE_OBS = 3
QUAD_POSIT_SIZE = 3

# Environments
NUM_TOT_ENVS = 18
ENVIRONMENTS_ID = {0: 'house_garden',
                   1: 'baxall',
                   2: 'church',
                   3: 'fire_training_center',
                   4: 'wood_island',
                   5: 'rochester_warehouse',
                   6: 'garden_of_eden',
                   7: 'buffalo_construction_site',
                   8: 'construction_site',
                   9: 'essex_castle',
                   10: 'fraser_gunnery_range',
                   11: 'hartley_mansion',
                   12: 'tin_hau_temple',
                   13: 'xcell_aerial_hq',
                   14: 'irchel',
                   15: 'hofdi_house'
                   }
                   
TESTING_ENVS_IDS = [9, 10, 11, 12, 13, 14, 15]

# Done Reasons
from mlgym import DoneReason

DONE_REASONS = [name for name, _ in DoneReason.__members__.items()]
