class SrdModel:
    model_type_code = 1
    # options: 0 - 'not configured',
    # 1 - 'exact'
    # 2 - 'OrtegaSpong' -with parameters as inputs, regressor - style
    # 3 - 'numeric'

    # # # # # # # # # # #
    # function handles

    MechanicalEquations_Numeric = []

    # # # # # # # # # # #
    # state

    theta = [] # Ortega - Spong
    model
    parameters - actual
    value
    estimated_theta = [] # Ortega - Spong
    model
    parameters - estimate

    numeric_functions_updated_time = 0
    numeric_functions_value = []

    g_dynamics_JSIM = []
    g_dynamics_RHS = []
    g_dynamics_ControlMap = []
    g_dynamics_LagrangeMultiplier_ConstraintJacobian = []
    ForcesForComputedTorqueController = []

    def __init__(self):

