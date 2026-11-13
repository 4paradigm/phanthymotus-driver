import unittest

from game import Choice, Game, POSES, outcome, parse_player_choice


class GameTests(unittest.TestCase):
    def test_all_nine_outcomes(self):
        cases = [
            (Choice.ROCK, Choice.ROCK, "draw"),
            (Choice.ROCK, Choice.PAPER, "player_win"),
            (Choice.ROCK, Choice.SCISSORS, "robot_win"),
            (Choice.PAPER, Choice.ROCK, "robot_win"),
            (Choice.PAPER, Choice.PAPER, "draw"),
            (Choice.PAPER, Choice.SCISSORS, "player_win"),
            (Choice.SCISSORS, Choice.ROCK, "player_win"),
            (Choice.SCISSORS, Choice.PAPER, "robot_win"),
            (Choice.SCISSORS, Choice.SCISSORS, "draw"),
        ]
        for robot, player, expected in cases:
            with self.subTest(robot=robot, player=player):
                self.assertEqual(outcome(robot, player), expected)

    def test_spoken_reports_are_unambiguous(self):
        for phrase, expected in [("我出石头。", Choice.ROCK), ("我出剪刀", Choice.SCISSORS),
                                 ("布！", Choice.PAPER), ("我出了布", Choice.PAPER)]:
            self.assertEqual(parse_player_choice(phrase), expected)
        for phrase in ["我不出石头", "不是剪刀", "石头还是布", "石头剪刀布", "出布吗？", "开始游戏", "", None]:
            with self.subTest(phrase=phrase), self.assertRaises(ValueError):
                parse_player_choice(phrase)

    def test_robot_commits_before_player_and_does_not_redraw_on_invalid_report(self):
        calls = []
        def choose(options):
            calls.append(options)
            return Choice.SCISSORS
        game = Game(choose)
        receipt = game.begin_round()
        self.assertEqual(len(calls), 1)
        self.assertNotIn("robot", receipt)
        with self.assertRaises(ValueError):
            game.receive_player("不是布")
        result = game.receive_player("我出布")
        self.assertEqual(len(calls), 1)
        self.assertEqual(result.robot, Choice.SCISSORS)
        self.assertEqual(result.outcome, "robot_win")
        self.assertEqual(result.requested_pose, "feet_staggered")

    def test_one_report_per_round_and_scores(self):
        game = Game(lambda _: Choice.ROCK)
        with self.assertRaises(RuntimeError):
            game.receive_player("布")
        game.begin_round()
        with self.assertRaises(RuntimeError):
            game.begin_round()
        game.receive_player("布")
        with self.assertRaises(RuntimeError):
            game.receive_player("布")
        game.begin_round()
        game.receive_player("剪刀")
        self.assertEqual(game.scores, {"robot_win": 1, "player_win": 1, "draw": 0})

    def test_stop_clears_pending_round(self):
        game = Game(lambda _: Choice.ROCK)
        game.begin_round()
        game.stop()
        with self.assertRaises(RuntimeError):
            game.receive_player("石头")
        with self.assertRaises(RuntimeError):
            game.begin_round()

    def test_pose_mapping_matches_confirmed_rules(self):
        self.assertEqual(POSES, {Choice.ROCK: "feet_together", Choice.PAPER: "feet_apart", Choice.SCISSORS: "feet_staggered"})


if __name__ == "__main__":
    unittest.main(verbosity=2)
