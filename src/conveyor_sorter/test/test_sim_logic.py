"""Headless end-to-end check of spawner + sensors + arms + stats."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import fake_ros  # noqa: E402


def test_sorting_is_correct():
    spawner, arm1, arm2, stats, logs = fake_ros.simulate(120.0)
    s = stats.stats
    assert s['spawned'] > 30
    assert s['red_bin'] > 0 and s['blue_bin'] > 0
    assert s['wrong_bin'] == 0
    for box in spawner.boxes.values():
        if box.state == 'BINNED' and box.bin in ('red', 'blue'):
            assert box.color == box.bin            # red only in red bin, blue only in blue bin
    assert not [m for lvl, m in logs if lvl == 'W']  # no IK failures / aborts
    # every black box that finished went to scrap, never to an arm bin
    assert all(b.bin == 'scrap' for b in spawner.boxes.values()
               if b.color == 'black' and b.state == 'BINNED')
