%% DMP_eo class
%  For encoding Cartesian Orientation.
%


classdef DMP_eo < handle
    
    properties (Constant)
        
          %% enum ARGUMENT
          
          %% enum TRAIN_METHOD
          LWR = 201;
          LS = 203;

    end
       
    properties
        
        dmp % 3x1 vector of DMP producing the desired trajectory for each x, y and z axes of the orientation error.

        can_clock_ptr % handle (pointer) to the canonical clock
        shape_attr_gating_ptr % pointer to gating function for the shape attractor
        
        ddeo % second derivative of the orientation error
        deo % first derivative of the orientation error
        dx % pahse variable derivative
    end

    methods
        %% DMP_eo constructor.
        %  @param[in] N_kernels: 3x3 matrix where each row contains the kernels for position,
        %                       velocity and acceleration and each row for x, y and z coordinate.
        %                       If 'N_kernels' is a column vector then every coordinate will have the
        %                       same number of kernels.
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
        %  @param[in] shape_attr_gating_ptr: Pointer to a DMP gating function object.
        %
        function this = DMP_eo(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr, dmp_type)
            
            if (nargin < 6), dmp_type = DMP.DMP_STD; end
            
            if (isscalar(N_kernels)), N_kernels = ones(3,1)*N_kernels; end
            if (isscalar(a_z)), a_z = ones(3,1)*a_z; end
            if (isscalar(b_z)), b_z = ones(3,1)*b_z; end

            this.can_clock_ptr = can_clock_ptr;
            this.shape_attr_gating_ptr = shape_attr_gating_ptr;

            for i=1:3
                this.dmp{i} = DMP(DMP.Az,a_z(i), DMP.Bz,b_z(i), DMP.NKernels,N_kernels(i), ...
                    DMP.CanClock,can_clock_ptr, DMP.ShapeAttrGat,shape_attr_gating_ptr, DMP.TYPE, dmp_type);
            end

        end
        
        
        %% Trains the DMP_eo.
        %  @param[in] train_method: The training method (see dmp_::TRAIN_METHOD enum).
        %  @param[in] Time: 1xN row vector with the timestamps of the training data points.
        %  @param[in] Quat_data: 3xN matrix with desired position at each column for each timestep.
        %  @param[in] rotVel_data: 3xN matrix with desired velocity at each column for each timestep.
        %  @param[in] rotAccel_data: 3xN matrix with desired acceleration at each column for each timestep.
        %  @param[in] train_error: Optinal pointer to return the training error as norm(F-Fd)/n_data.
        function train_err = train(this, train_method, Time, Quat_data, rotVel_data, rotAccel_data)

            tau = Time(end);
            this.setTau(tau);
            
            if (train_method == DMP_eo.LWR), dmp_train_method = DMP.LWR;
            elseif (train_method == DMP_eo.LS), dmp_train_method = DMP.LS;  
            else, error('[DMP_eo::train]: Unsopported training method...');
            end
            
            n_data = length(Time);
            eo_data = zeros(3, n_data);
            deo_data = zeros(3, n_data);
            ddeo_data = zeros(3, n_data);
            
            Qg = Quat_data(:,end);
            
            for j=1:n_data
               eo_data(:,j) = quatLog( quatProd(Qg, quatInv(Q) ) );
               deo_data(:,j) = this.rotVel2deo(rotVel_data(:,j), Quat_data(:,j), Qg);
            end
            
            Ts = Time(end) - Time(end-1);
            dTime = [diff(Time) Ts];
            for i=1:3
                ddeo_data(i,:) = [diff(deo_data(i,:)) 0] ./ dTime;
            end
            
                
            if (nargout > 0)
                train_err = zeros(3,1);
                for i=1:3
                    train_err(i) = this.dmp{i}.train(dmp_train_method, Time, eo_data(i,:), deo_data(i,:), ddeo_data(i,:));
                end
            else
                for i=1:3
                    this.dmp{i}.train(dmp_train_method, Time, eo_data(i,:), deo_data(i,:), ddeo_data(i,:));
                end
            end

        end
        
        %% Calculates the derivatives of the DMP states. The derivatives can then be
        %% retrieved with 'getDx', 'getdeo' and 'getddeo'.
        %  @param[in] x: phase variable.
        %  @param[in] Y: 'y' state of the DMP.
        %  @param[in] Z: 'z' state of the DMP.
        %  @param[in] Y0: Initial position.
        %  @param[in] Yg: Goal position.
        %  @param[in] Y_c: Coupling term for the deonamical equation of the 'y' state.
        %  @param[in] Z_c: Coupling term for the deonamical equation of the 'z' state.
        function calcStatesDot(this, x, Y, Z, Y0, Yg, Y_c, Z_c)
            
            if (nargin < 7), Y_c=zeros(3,1); end
            if (nargin < 8), Z_c=zeros(3,1); end
            
            for i=1:3, this.dmp{i}.calcStatesDot(x, Y(i), Z(i), Y0(i), Yg(i), Y_c(i), Z_c(i)); end
            
            this.ddeo = zeros(3,1);
            this.deo = zeros(3,1);
            for i=1:3
                this.deo(i) = this.dmp{i}.getdeo();
                this.ddeo(i) = this.dmp{i}.getddeo();    
            end
            this.dx = this.phaseDot(x);

        end
        
        
        %% Returns the acceleration for the given input state defined by the timestamp,
        %  the orientation, the angular velocity and acceleration, the initial and target orientation
        %  and an optinal coupling term.
        %  @param[in] x: phase variable.
        %  @param[in] P: Current position.
        %  @param[in] dP: Current velocity.
        %  @param[in] P0: Initial position.
        %  @param[in] Pg: Goal position.
        %  @param[in] Z_c: Coupling term. (optional, default=arma::vec().zeros(3))
        %  @return ddP: Acceleration.
        %
        function ddP = getAccel(this, x, P, dP, P0, Pg, Z_c)
            
            if (nargin < 7), Z_c=zeros(3,1); end
            ddP = zeros(3,1);
            for i=1:3, ddP(i) = this.dmp{i}.getAccel(x, P(i), dP(i), P0(i), Pg(i), Z_c(i)); end
            
        end
        

        %% Returns the time scaling factor.
        %  @return: The time scaling factor.
        %
        function tau = getTau(this)

            tau = this.can_clock_ptr.getTau();

        end
        
        
        %% Sets the time scaling factor.
        %  @param[in] tau: The time scaling factor.
        %
        function setTau(this, tau)

            this.can_clock_ptr.setTau(tau);

        end
        
        
        %% Returns the phase variable corresponding to the given time instant.
        %  @param[in] t: The time instant.
        %  @return: The phase variable for time 't'.
        %
        function x = phase(this, t)
            
            x = this.can_clock_ptr.getPhase(t);

        end
        
        
        %% Returns the derivative of the phase variable.
        %  @param[in] x: The phase variable.
        %  @return: The derivative of the phase variable.
        %
        function dx = phaseDot(this, x)
            
            dx = this.can_clock_ptr.getPhaseDot(x);

        end
        
        function dx = getDx(this), dx=this.dx; end
        function deo = getdeo(this), deo=this.deo; end
        function ddeo = getddeo(this), ddeo=this.ddeo; end
        
    end
    
    methods (Static)
        
        %% Given the a quaternion, its derivative and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] dQ: The quaternion derivative.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        function deo = dquat2deo(dQ, Q, Qg)
            
            invQ = quatInv(Q);
            Q_delta = quatProd(Qg, invQ);
            rotVel = 2 * quatProd(dQ, invQ);
            J_deo_dQ = dquat2deoJacob(Q_delta);
            deo = -0.5*J_deo_dQ * quatProd(Q_delta, rotVel);

        end
        
        
        %% Given the a quaternion, its rotational velocity and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        function deo = rotVel2deo(rotVel, Q, Qg)
            
            invQ = quatInv(Q);
            Q_delta = quatProd(Qg, invQ);
            J_deo_dQ = dquat2deoJacob(Q_delta);
            deo = -0.5*J_deo_dQ * quatProd(Q_delta, [0; rotVel]);

        end
        
        
        
        %% Returns the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @return: Jacobian from derivative of orientation error to quaternion derivative.
        %
        function J_dQ_deo = deo2dquatJacob(Q)

            zero_tol = 1e-16;
            w = Q(1);
            v = Q(2:4);
            norm_v = norm(v);
            eta = v / (norm_v + zero_tol);
            theta = 2*atan2(norm_v, w);
            Eta = eta*eta';
            sin_theta_2 = sin(theta/2);
            
            J_dQ_deo = zeros(4,3);
            J_dQ_deo(1,:) = -0.5 * sin_theta_2 * eta';
            J_dQ_deo(2:4,:) = (eye(3,3) - Eta)*sin_theta_2/(theta + zero_tol) + 0.5*cos(theta/2)*Eta;

        end
        
        
        %% Returns the time derivative of the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @param[in] deo: The derivative of the orientation error.
        %  @return: The derivative of the Jacobian from derivative of orientation error to quaternion derivative.
        %
        function dJ_dQ_deo = deo2dquatJacobDot(Q, deo)

            zero_tol = 1e-16;
            w = Q(1);
            v = Q(2:4);
            norm_v = norm(v);
            eta = v / (norm_v + zero_tol);
            theta = 2*atan2(norm_v, w);
            Eta = eta*eta';
            sin_theta_2 = sin(theta/2);
            
            dJ_dQ_deo = zeros(4,3);
            dJ_dQ_deo(1,:) = -0.5 * sin_theta_2 * eta';
            dJ_dQ_deo(2:4,:) = (eye(3,3) - Eta)*sin_theta_2/(theta + zero_tol) + 0.5*cos(theta/2)*Eta;

        end
        
        %% Returns the Jacobian from the quaternion derivative to the derivative of the orientation error.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @return: Jacobian from quaternion derivative to derivative of orientation error.
        %
        function J_deo_dQ = dquat2deoJacob(Q)

            zero_tol = 1e-16;
            w = Q(1);
            v = Q(2:4);
            norm_v = norm(v);
            eta = v / (norm_v + zero_tol);
            theta = 2*atan2(norm_v, w);
            sin_theta_2 = sin(theta/2);
            
            J_deo_dQ = zeros(3,4);
            J_deo_dQ(:,1) = eta*(theta*cos(theta/2) - 2*sin_theta_2)/(sin_theta_2^2 + zero_tol);
            J_deo_dQ(:,2:4) = eye(3,3)*theta/sin_theta_2;

        end
        
    end
    
end
