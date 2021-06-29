import imp
from casadi import *
import numpy as np
from SrdPy import get
from SrdPy.Controllers import LTI_CLQE
from SrdPy.Controllers.LTI_CLQE import LTI_System
tol = 10**(-5)

# [0, 0, phi_1, phi_2]
q = [0, 0, 0, 0]
v = np.zeros(len(q))
x = vertcat(q,v)
u = 0

Handler_dynamics_generalized_coordinates_model = get('handlerGeneralizedCoordinatesModel')
Handler_dynamics_Linearized_Model              = get('handlerLinearizedModel')
Handler_Constraints_Model                      = get('handlerConstraints')


k = Handler_Constraints_Model.dofConstraint
dof = Handler_dynamics_generalized_coordinates_model.dofConfigurationSpaceRobot
m = Handler_dynamics_generalized_coordinates_model.dofControl

H = Handler_dynamics_generalized_coordinates_model.getJointSpaceInertiaMatrix(q)
c = Handler_dynamics_generalized_coordinates_model.getBiasVector(q, v)
T = Handler_dynamics_generalized_coordinates_model.getControlMap(q)

F  = Handler_Constraints_Model.getJacobian(q)
dF = Handler_Constraints_Model.getJacobianDerivative(q,v)

iH = pinv(H)
f0 = vertcat(v,iH@(T*u - c))

M =  vertcat(horzcat(H, -F.T),
     horzcat(F, SX.zeros(k,k)))
iM = pinv(M)

A = Handler_dynamics_Linearized_Model.getA(q, v, u, iM)
B = Handler_dynamics_Linearized_Model.getB(q, v,    iM)

g = f0 - A @ x - B @ u

G = vertcat(horzcat(SX.zeros(k, dof), F),
            horzcat(F, dF))



C_case0 = np.array([ 0, 0, 0, 1, 0, 0, 0, 0 ] )
 
C_case1 = np.array([
    [0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 1]])
 
C_case2 = np.array([
     [1, 0, 0, 0, 0, 0, 0, 0],
     [0, 1, 0, 0, 0, 0, 0, 0],
     [0, 0, 1, 0, 0, 0, 0, 0],
     [0, 0, 0, 1, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 0, 1, 0],
     [0, 0, 0, 0, 0, 0, 0, 1]])
 
C_case3 = np.array([
     [1, 0, 0, 0, 0, 0, 0, 0],
     [0, 1, 0, 0, 0, 0, 0, 0],
     [0, 0, 1, 0, 0, 0, 0, 0],
     [0, 0, 0, 1, 0, 0, 0, 0],
     [0, 0, 0, 0, 0, 1, 0, 0],
     [0, 0, 0, 0, 0, 0, 0, 1]])

C_case4 = np.eye(2*dof) 

C = C_case1
 

ControllerCost = {'Q': 100*np.eye(4), 'R': 1}
ObserverCost   = {'Q': 100*np.eye(4), 'R': np.array([[C.shape[0]]])}
system = LTI_System(A, B,  C,  G, g,ControllerCost,  ObserverCost,
    0.01*np.array([0,0,np.random.randn(),np.random.randn(), 0,0,0,0]), 
    0.01*np.array([0,0,np.random.randn(),np.random.randn(), 0,0,0,0]),
    tol)

Output = LTI_CLQE(system)

'''

Time = 15
[TOUT,YOUT] = ode45(Output.closed_loop.z_xi.ode_fnc, [0 Time], Output.closed_loop.z_xi.Y0)

z        = YOUT(:, 1:Output.sizes.size_z)
z_est    = YOUT(:, (1+Output.sizes.size_z):(2*Output.sizes.size_z))
zeta_est = YOUT(:, (1+2*Output.sizes.size_z):(2*Output.sizes.size_z+Output.sizes.size_zeta) )
x_est = (Output.Matrices.N *z_est' + Output.Matrices.R_used*zeta_est')'

figure('Color', 'w')
subplot(2, 2, 1)
plot(TOUT, z, 'LineWidth', 1.5) hold on title('z')
plot([TOUT(1) TOUT(end)], [Output.desired.z_corrected' Output.desired.z_corrected'], '--', 'LineWidth', 0.8)

subplot(2, 2, 2)
plot(TOUT, z_est) title('z est')
subplot(2, 2, 3)
if ~isempty(zeta_est)
    plot(TOUT,zeta_est) title('zeta est')
end

subplot(2, 2, 4)

plot(TOUT, x_est, 'LineWidth', 1.5) hold on title('x est')
plot([TOUT(1) TOUT(end)], [Output.desired.x_corrected' Output.desired.x_corrected'], '--', 'LineWidth', 0.8)


drawnow
##################################################################

[TOUT,YOUT] = ode45(Output.closed_loop.x_xi.ode_fnc, [0 Time], Output.closed_loop.x_xi.Y0)

x        = YOUT(:, 1:Output.sizes.size_x)
z_est    = YOUT(:, (1+Output.sizes.size_x):(Output.sizes.size_x+Output.sizes.size_z))
zeta_est = YOUT(:, (1+Output.sizes.size_x+Output.sizes.size_z):(Output.sizes.size_x+Output.sizes.size_z+Output.sizes.size_zeta) )
z_calc = (Output.Matrices.N'*x')'

figure('Color', 'w')
subplot(3, 2, 1)
plot(TOUT, x, 'LineWidth', 1.5) hold on title('x')
plot([TOUT(1) TOUT(end)], [Output.desired.x_corrected' Output.desired.x_corrected'], '--', 'LineWidth', 0.8)

subplot(3, 2, 2)
plot(TOUT, z_est) title('z est')
subplot(3, 2, 3)

if ~isempty(zeta_est)
    plot(TOUT, zeta_est) title('zeta est')
end

subplot(3, 2, 4)
plot(TOUT, z_calc) hold on title('z calc')
plot([TOUT(1) TOUT(end)], [Output.desired.z_corrected' Output.desired.z_corrected'], '--', 'LineWidth', 0.8)
subplot(3, 2, 5)
plot(TOUT, z-z_calc) title('z-z calc')

drawnow

'''