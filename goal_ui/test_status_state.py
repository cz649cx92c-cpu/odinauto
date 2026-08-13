#!/usr/bin/env python3
import unittest

from status_state import derive_navigation_state


class NavigationStateTest(unittest.TestCase):
    def state(self, **overrides):
        values = dict(target_active=False, planner_active=False, obstacle=False,
                      outcome='idle', has_route=False)
        values.update(overrides)
        return derive_navigation_state(**values)

    def test_active_obstacle_has_highest_priority(self):
        self.assertEqual(self.state(target_active=True, planner_active=True,
                                    obstacle=True)['code'], 'blocked')

    def test_active_without_planner_waits(self):
        self.assertEqual(self.state(target_active=True)['code'], 'waiting_path')

    def test_active_planner_is_planning(self):
        self.assertEqual(self.state(target_active=True, planner_active=True)['code'],
                         'planning')

    def test_completed_route_survives_empty_plan(self):
        self.assertEqual(self.state(outcome='completed', has_route=True)['code'],
                         'completed')

    def test_manual_stop_is_distinct(self):
        self.assertEqual(self.state(outcome='stopped')['code'], 'stopped')


if __name__ == '__main__':
    unittest.main()
