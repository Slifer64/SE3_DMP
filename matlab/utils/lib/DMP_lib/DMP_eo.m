%% DMP_eo class
%  For encoding Cartesian Orientation.
%


classdef DMP_eo < handle
    
    properties (Constant)
          
          %% enum TRAIN_METHOD
          LWR = 201;
          LS = 203;

    end
       
    properties
        
        dmp % 3x1 vector of DMP producing the desired trajectory for each x, y and z axes of the orientation error.

        can_clock_ptr % handle (pointer) to the canonical clock
        shape_attr_gating_ptr % pointer to gating function for the shape attractor
        
        dZ % second derivative of the orientation error
        dY % first derivative of the orientation error
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
               Q = Quat_data(:,j);
               Qe = quatProd(Qg, quatInv(Q) );
               eo_data(:,j) = quatLog(Qe);
               deo_data(:,j) = this.rotVel2deo(rotVel_data(:,j), Qe);
               ddeo_data(:,j) = this.rotAccel2ddeo(rotAccel_data(:,j), rotVel_data(:,j), Qe);
            end
            
            
            Ts = Time(end) - Time(end-1);
            dTime = [diff(Time) Ts];
            for i=1:3
%                 deo_data2(i,:) = [diff(eo_data(i,:)) 0] ./ dTime;
                ddeo_data(i,:) = [diff(deo_data(i,:)) 0] ./ dTime;
            end
            
%             deo_diff = abs(deo_data-deo_data2);
%             figure;
%             plot(deo_diff');
% 
%             ddeo_diff = abs(ddeo_data-ddeo_data2);
%             figure;
%             plot(Time, ddeo_data(1,:), Time, ddeo_data2(1,:));
%             error('stop')

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
        %  @param[in] Y_c: Coupling term for the deonamical equation of the 'y' state.
        %  @param[in] Z_c: Coupling term for the deonamical equation of the 'z' state.
        function calcStatesDot(this, x, Y, Z, Y0, Y_c, Z_c)
            
            if (nargin < 6), Y_c=zeros(3,1); end
            if (nargin < 7), Z_c=zeros(3,1); end
            
            for i=1:3, this.dmp{i}.calcStatesDot(x, Y(i), Z(i), Y0(i), 0, Y_c(i), Z_c(i)); end
            
            this.dY = zeros(3,1);
            this.dZ = zeros(3,1);
            for i=1:3
                this.dY(i) = this.dmp{i}.getDy();
                this.dZ(i) = this.dmp{i}.getDz();    
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
        function dy = getDy(this), dy=this.dY; end
        function dz = getDz(this), dz=this.dZ; end
        
    end
    
    methods (Static)
        
        %% Given a quaternion, its derivative and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] dQ: The quaternion derivative.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        function deo = dquat2deo(dQ, Qe)
            
            rotVel = 2 * quatProd(dQ, invQ);
            J_deo_dQ = jacobDeoDquat(Qe);
            deo = -0.5*J_deo_dQ * quatProd(Qe, rotVel);

        end
        
        
        %% Given a quaternion, its rotational velocity and a target quaternion, returns
        %% the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The derivative of the orientation error.
        %
        function deo = rotVel2deo(rotVel, Qe)
            
            J_deo_dQ = DMP_eo.jacobDeoDquat(Qe);
            deo = -0.5*J_deo_dQ * quatProd(Qe, [0; rotVel]);

        end
        
        
        %% Given a quaternion, its rotational velocity and acceleration and a target quaternion,
        %% returns the second derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotAccel: The quaternion's rotational acceleration.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The second derivative of the orientation error.
        %
        function ddeo = rotAccel2ddeo(rotAccel, rotVel, Qe)
            
            J_deo_dQ = DMP_eo.jacobDeoDquat(Qe);
            dJ_deo_dQ = DMP_eo.jacobDotDeoDquat(Qe, rotVel);
            
            rotVelQ = [0; rotVel];
            rotAccelQ = [0; rotAccel];
            
            ddeo = 0.5 * (dJ_deo_dQ * quatProd(Qe, rotVelQ) - J_deo_dQ * quatProd( Qe, rotAccelQ-0.5*quatProd(rotVelQ,rotVelQ) ) );

        end
        
        %% Given a quaternion, its rotational velocity and acceleration and a target quaternion,
        %% returns the second derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] rotAccel: The quaternion's rotational acceleration.
        %  @param[in] rotVel: The quaternion's rotational velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The second derivative of the orientation error.
        %
        function rotAccel = ddeo2rotAccel(ddeo, rotVel, Qe)
            
            deo = DMP_eo.rotVel2deo(rotVel, Qe);
            invQe = quatInv(Qe);
            J_dQ_deo = DMP_eo.jacobDquatDeo(Qe);
            dJ_dQ_deo = DMP_eo.jacobDotDquatDeo(Qe, rotVel);
            
            rotVelQ = [0; rotVel];
            
            rotAccel = -quatProd(quatProd(rotVelQ, invQe), J_dQ_deo*deo) - 2*quatProd(invQe, dJ_dQ_deo*deo) - 2*quatProd(Qe, J_dQ_deo*ddeo);
            rotAccel = rotAccel(2:4);
            
        end
        
        
        %% Given a quaternion, a target quaternion and the quaternion error velocity w.r.t the
        %% target, returns the derivative of the orientation error w.r.t the target quaternion.
        %  @param[in] deo: Quaternion error velocity.
        %  @param[in] Q: The quaternion.
        %  @param[in] Qg: The target quaternion.
        %  @return: The rotational velocity of the quaternion.
        %
        function rotVel = deo2rotVel(deo, Qe)
            
            J_dQ_deo = DMP_eo.jacobDquatDeo(Qe);
            rotVel = -2 * quatProd( quatInv(Qe), J_dQ_deo*deo );
            rotVel = rotVel(2:4);
            
        end
         
        
        %% Returns the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @return: Jacobian from derivative of orientation error to quaternion derivative.
        %
        function J_dQ_deo = jacobDquatDeo(Qe)

            if (abs(Qe(1)-1) <= 1e-10)
                J_dQ_deo = [zeros(1, 3); eye(3,3)];
                return;
            end

            w = Qe(1);
            v = Qe(2:4);
            norm_v = norm(v);
            eta = v / norm_v;
            theta = atan2(norm_v, w);
            Eta = eta*eta';
            s_th = sin(theta);
            c_th = cos(theta);
            
            J_dQ_deo = zeros(4,3);
            J_dQ_deo(1,:) = -0.5 * s_th * eta';
            J_dQ_deo(2:4,:) = 0.5 * ( (eye(3,3) - Eta)*s_th/theta + c_th*Eta );

        end
        
        %% Returns the Jacobian from the quaternion derivative to the derivative of the orientation error.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @return: Jacobian from quaternion derivative to derivative of orientation error.
        %
        function J_deo_dQ = jacobDeoDquat(Qe)
            
            if (abs(Qe(1)-1) <= 1e-10)
                J_deo_dQ = [zeros(3,1) eye(3,3)];
                return;
            end
            
            w = Qe(1);
            v = Qe(2:4);
            norm_v = norm(v);
            eta = v / norm_v;
            theta = atan2(norm_v, w);
            s_th = sin(theta);
            c_th = cos(theta);
            
            J_deo_dQ = zeros(3,4);
            J_deo_dQ(:,1) = 2*eta*(theta*c_th - s_th)/s_th^2;
            J_deo_dQ(:,2:4) = 2*eye(3,3)*theta/s_th;

        end
       
        %% Returns the time derivative of the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @param[in] deo: The derivative of the orientation error.
        %  @return: The derivative of the Jacobian from derivative of orientation error to quaternion derivative.
        %
        function dJ_deo_dQ = jacobDotDeoDquat(Qe, vRot)

            zero_tol = 1e-10;
            
            w = Qe(1);
            v = Qe(2:4);
            norm_v = norm(v);
            eta = v / (norm_v + zero_tol);
            th = atan2(norm_v, w); % it's the theta/2
            
            if (th < 1e-16), th = 1e-16; end
            
            Eta = eta*eta';
            s_th = sin(th);
            c_th = cos(th);
            temp = (th*c_th-s_th)/s_th^2;
            
            deo = DMP_eo.deo2rotVel(vRot, Qe);
            
            dJ_deo_dQ = zeros(3,4);
            dJ_deo_dQ(:,1) = 0.5*((-th/s_th - 2*c_th*temp/s_th)*Eta + temp*(eye(3,3)-Eta))*deo;
            dJ_deo_dQ(:,2:4) = -0.5*temp*eta'*deo*eye(3,3);
            dJ_deo_dQ = 2 *dJ_deo_dQ;

        end
        
        %% Returns the time derivative of the Jacobian from the derivative of orientation error to the quaternion derivative.
        %  @param[in] Q: The quaternion for which we want to calculate the Jacobian.
        %  @param[in] deo: The derivative of the orientation error.
        %  @return: The derivative of the Jacobian from derivative of orientation error to quaternion derivative.
        %
        function dJ_dQ_deo = jacobDotDquatDeo(Qe, vRot)

            zero_tol = 1e-10;
            
            w = Qe(1);
            v = Qe(2:4);
            norm_v = norm(v);
            eta = v / (norm_v + zero_tol);
            th = atan2(norm_v, w);
            Eta = eta*eta';
            I_eta = eye(3,3) - Eta;
            s_th = sin(th);
            c_th = cos(th);
            
            deo = DMP_eo.deo2rotVel(vRot, Qe);
            
            dJ_dQ_deo = zeros(4,3);
            dJ_dQ_deo(1,:) = -0.25 * deo' * (c_th*Eta + (s_th/th)*I_eta);
            dJ_dQ_deo(2:4,:) = 0.25*(eta'*deo)*( ((th*c_th-s_th)/th^2)*I_eta - s_th*Eta ) + 0.25*((c_th-s_th/th)/th)*( (eta*deo')*I_eta + I_eta*(deo*eta') );
            
            
            dJ_dQ_deo
            dJ_dQ_deo2 = DMP_eo.getJacobianAcceleration( 0.5*quatLog(Qe), 0.5*deo)
            
            pause
        end
        
        function J = getJacobianAcceleration( logQ, dLogQ)

            theta = norm(logQ);
            n = logQ / theta;

            J1 = -dLogQ' * (cos(theta) * n * n' + (eye(3) - n * n') * sin(theta) / theta);
            scalarJ2 = (1 / theta) * (cos(theta) - sin(theta) / theta);
            J2 = ((eye(3) - n * n') * dLogQ * n' + n * dLogQ' * (eye(3) - n * n'));

            J = [J1; scalarJ2 * J2];

        end
        
    end
    
end
