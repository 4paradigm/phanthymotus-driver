"""Every public shared lowcmd motion entry requires explicit operator consent."""
import unittest
from unittest import mock

import device


class UpperBodyConfirmationTests(unittest.TestCase):
    def cards(self):
        control = device.ArmControlPlugin({}, '', None)
        return control, device.WaistControlPlugin(control), device.HeadControlPlugin(control)

    def test_unconfirmed_motion_never_reaches_controller(self):
        control, waist, head = self.cards()
        arm_actions = ['reset', 'preset', 'set_joints',
                       *control._GROUP_JOINTS, *device.ARM_ACTIONS]
        for card, actions in ((control, arm_actions),
                              (waist, ['reset', 'set_angles']),
                              (head, ['reset', 'set_angles'])):
            for action in actions:
                for args in ({}, {'confirm': False}, {'confirm': None},
                             {'confirm': 0}, {'confirm': 1}, {'confirm': 'true'}):
                    with self.subTest(card=card.PREFIX, action=action, args=args), \
                         mock.patch.object(control, '_set_targets') as submit, \
                         mock.patch.object(control, '_ready_error') as ready, \
                         mock.patch.object(control, 'start') as start:
                        result = card.dispatch(action, args)
                        self.assertEqual('CONFIRMATION_REQUIRED', result['code'])
                        submit.assert_not_called()
                        ready.assert_not_called()
                        start.assert_not_called()

    def test_schema_marks_every_advertised_motion(self):
        for card in self.cards():
            schema = card.get_tool()['inputSchema']
            self.assertTrue(schema['x-is-dangerous'])
            self.assertFalse(schema['properties']['confirm']['default'])
            for action in schema['properties']['action']['enum']:
                entry = schema['x-action-params'][action]
                self.assertIn('confirm', entry['params'])
                self.assertIn('confirm', entry['required'])
                self.assertTrue(entry['x-is-dangerous'])

    def test_nonmotion_actions_remain_available(self):
        control, waist, head = self.cards()
        for card in (control, waist, head):
            with mock.patch.object(control, 'start') as start:
                self.assertEqual({'state': 'ready'}, card.dispatch('start', {}))
                start.assert_called_once()
            with mock.patch.object(control, '_stop_and_wait', return_value={'state': 'idle'}) as stop:
                self.assertEqual({'state': 'idle'}, card.dispatch('stop', {}))
                stop.assert_called_once()
            self.assertIn('state', card.dispatch('info', {}))
        self.assertIn('state', control.dispatch('get_state', {}))


if __name__ == '__main__':
    unittest.main()
