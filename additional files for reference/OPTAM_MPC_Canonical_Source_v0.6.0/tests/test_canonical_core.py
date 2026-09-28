import unittest

from optam_mpc.controller import (
    ControlledVariableConfiguration,
    GenericNMPCConfiguration,
    GenericNonlinearMPC,
    ManipulatedMoveCombinationConfiguration,
    ManipulatedVariableConfiguration,
)


class IntegratingModel:
    disturbance_dimension = 0
    output_dimension = 1
    state_dimension = 1

    def __init__(self, input_count):
        self.manipulated_input_dimension = input_count

    def transition_values(self, state, inputs, disturbances):
        del disturbances
        return (state[0] + sum(inputs),)

    def output_values(self, state, inputs, disturbances):
        del inputs, disturbances
        return (state[0],)


def make_controller(input_count, coefficients=None):
    combinations = () if coefficients is None else (
        ManipulatedMoveCombinationConfiguration(
            name="declared combination",
            coefficients=tuple(coefficients),
            weight=2.0,
            normalization=1.0,
        ),
    )
    return GenericNonlinearMPC(
        IntegratingModel(input_count),
        GenericNMPCConfiguration(
            prediction_horizon_samples=2,
            control_horizon_samples=1,
            controlled_variables=(ControlledVariableConfiguration.for_target(
                "level", 0, target=1.0, weight=1.0, normalization=1.0,
                rate_weight=0.5, rate_normalization=1.0,
            ),),
            manipulated_variables=tuple(
                ManipulatedVariableConfiguration(
                    f"mv_{index}", -2.0, 2.0, 1.0, 0.1, 1.0,
                ) for index in range(input_count)
            ),
            move_combinations=combinations,
        ),
    )


class DimensionIndependentMoveCombinationTests(unittest.TestCase):
    def test_declared_combinations_work_for_one_two_and_three_mvs(self):
        for coefficients in ((1.0,), (0.5, -0.5), (1.0, -1.0, 0.25)):
            with self.subTest(dimension=len(coefficients)):
                controller = make_controller(len(coefficients), coefficients)
                result = controller.calculate_move((0.0,), (), (0.0,) * len(coefficients))
                self.assertTrue(result.success)

    def test_coefficient_count_must_match_model(self):
        with self.assertRaisesRegex(ValueError, "requires exactly 3 coefficients"):
            make_controller(3, (1.0, -1.0))

    def test_online_tuning_does_not_rebuild_solver(self):
        controller = make_controller(2, (0.5, 0.5))
        solver_before = controller._solver
        controller.update_tuning(
            cv_weights={0: 3.0},
            cv_rate_weights={0: 0.75},
            cv_targets={0: 1.5},
            mv_move_weights={1: 0.4},
            move_combination_weights={0: 4.0},
        )
        self.assertIs(solver_before, controller._solver)
        result = controller.calculate_move((0.0,), (), (0.0, 0.0))
        self.assertTrue(result.success)

    def test_invalid_runtime_update_rolls_back(self):
        controller = make_controller(1, (1.0,))
        previous = controller.configuration
        with self.assertRaises(ValueError):
            controller.update_tuning(mv_move_weights={0: -1.0})
        self.assertEqual(previous, controller.configuration)


if __name__ == "__main__":
    unittest.main()
