# Feature profiles (realism-2 design G3, sections 7 and 8): the three reference profiles
# premium_fr3, mid_kr1205 and budget_xarm7 load with the values of design section 7 (per-joint
# expansions of the brief), unknown keys in the new groups are rejected naming them, and the
# realism-2 sweep files load with every param resolving.
cobot_add_test(test_config_profiles test_config_profiles.cpp test_profile_sweeps.cpp)
