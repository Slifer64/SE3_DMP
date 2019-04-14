%% DMP_orient class
%  For encoding Cartesian Orientation.
%


classdef DMP_orient < handle
    
    properties (Constant)
        
          %% enum ARGUMENT
          
          %% enum TRAIN_METHOD
          LWR = 201;
          LS = 203;

    end
       
    properties
        
        Qgd % Trained target orientation as unit quaternion.
        Q0d % Trained initial orientation as unit quaternion.
        log_Qgd_invQ0d % log(Qgd * inv(Q0d)), precalculated
        tau_d % Trained time-scaling.
        
        eq_f % 3x1 vector of WSoG producing the desired orientation.
        vRot_f % 3x1 vector of WSoG producing the desired rotational velocity.
        dvRot_f % 3x1 vector of WSoG producing the desired rotational acceleration.
        
        a_z % parameter 'a_z' relating to the spring-damper system
        b_z % parameter 'b_z' relating to the spring-damper system

        can_clock_ptr % handle (pointer) to the canonical clock
        shape_attr_gating_ptr % pointer to gating function for the shape attractor

        zero_tol % tolerance value used to avoid divisions with very small numbers

    end

    methods
        %% DMP_orient constructor.
        %  @param[in] N_kernels: 3x3 matrix where each row contains the kernels for position,
        %                       velocity and acceleration and each row for x, y and z coordinate.
        %                       If 'N_kernels' is a column vector then every coordinate will have the
        %                       same number of kernels.
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
        %  @param[in] shape_attr_gating_ptr: Pointer to a DMP gating function object.
        %
        function this = DMP_orient(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr)

            this.zero_tol = 1e-30; %realmin;
            this.a_z = a_z;
            this.b_z = b_z;
            this.can_clock_ptr = can_clock_ptr;
            this.shape_attr_gating_ptr = shape_attr_gating_ptr;

            if (isscalar(N_kernels)), N_kernels = ones(3,1)*N_kernels; end
            if (size(N_kernels,2) == 1), N_kernels = repmat(N_kernels,1,3); end

            shapeAttrGatFun_ptr = @(x)getOutput(this.shape_attr_gating_ptr,x);
            for i=1:3
                this.eq_f{i} = WSoG(N_kernels(1,i), shapeAttrGatFun_ptr);
                this.vRot_f{i} = WSoG(N_kernels(2,i), shapeAttrGatFun_ptr);
                this.dvRot_f{i} = WSoG(N_kernels(3,i), shapeAttrGatFun_ptr);
            end

        end
        
        
        %% Trains the DMP_orient.
        %  @param[in] train_method: The training method (see dmp_::TRAIN_METHOD enum).
        %  @param[in] Time: 1xN row vector with the timestamps of the training data points.
        %  @param[in] Qd_data: 4xN matrix with desired orientation as unit quaternion at each column for each timestep.
        %  @param[in] vRotd_data: 3xN matrix with desired angular velocity at each column for each timestep.
        %  @param[in] dvRotd_data: 3xN matrix with desired angular acceleration at each column for each timestep.
        %  @param[in] train_error: Optinal pointer to return the training error as norm(F-Fd)/n_data.
        %
        function train_err = train(this, train_method, Time, Qd_data, vRotd_data, dvRotd_data)

            n_data = length(Time);
            
            this.Q0d = Qd_data(:,1);
            this.Qgd = Qd_data(:,end);
            this.log_Qgd_invQ0d = this.quatLog( this.quatProd(this.Qgd, this.quatInv(this.Q0d) ) );
            
            eqd = zeros(3, n_data);           
            for j=1:n_data, eqd(:,j) = this.quatLog(this.quatProd(Qd_data(:,j),this.quatInv(this.Qgd))); end

            tau = Time(end);
            this.tau_d = tau;
            this.setTau(tau);
    
            x = this.phase(Time);
            
            if (train_method == DMP_orient.LWR), wsog_train_method = WSoG.LWR;
            elseif (train_method == DMP_orient.LS), wsog_train_method = WSoG.LS;  
            else, error('[DMP_orient::train]: Unsopported training method...');
            end
                
            if (nargout > 0)
                train_err = zeros(3,3);
                for i=1:3
                    train_err(1,i) = this.eq_f{i}.train(wsog_train_method, x, eqd(i,:));
                    train_err(2,i) = this.vRot_f{i}.train(wsog_train_method, x, vRotd_data(i,:));
                    train_err(3,i) = this.dvRot_f{i}.train(wsog_train_method, x, dvRotd_data(i,:));
                end
            else
                for i=1:3
                    this.eq_f{i}.train(wsog_train_method, x, eqd(i,:));
                    this.vRot_f{i}.train(wsog_train_method, x, vRotd_data(i,:));
                    this.dvRot_f{i}.train(wsog_train_method, x, dvRotd_data(i,:));
                end
            end

        end
        
        
        %% Returns the angular acceleration for the given input state defined by the timestamp,
        %  the orientation, the angular velocity and acceleration, the initial and target orientation
        %  and an optinal coupling term.
        %  @param[in] x: phase variable.
        %  @param[in] Q: Current orientation as unit quaternion.
        %  @param[in] vRot: Current rotational velocity.
        %  @param[in] Q0: initial orientation as unit quaternion.
        %  @param[in] Qg: Goal orientation as unit quaternion.
        %  @param[in] y_c: Coupling term. (optional, default=arma::vec().zeros(3))
        %  @return dvRot: Rotational acceleration.
        %
        function dvRot = getRotAccel(this, x, Q, vRot, Q0, Qg, y_c)

            if (nargin < 7), y_c=zeros(3,1); end

            eqd = zeros(3,1);
            vRotd = zeros(3,1);
            dvRotd = zeros(3,1);
            
            for i=1:3
                eqd(i) = this.eq_f{i}.output(x);
                vRotd(i) = this.vRot_f{i}.output(x);
                dvRotd(i) = this.dvRot_f{i}.output(x);
            end 

            Qd = this.quatProd( this.quatExp(eqd), this.Qgd );
            
            tau = this.getTau();
            kt = this.tau_d / tau;
            ks = this.quatLog( this.quatProd(Qg, this.quatInv(Q0) ) ) ./ this.log_Qgd_invQ0d;
            
            QQg = this.quatProd(Q,this.quatInv(Qg));
            inv_exp_QdQgd = this.quatInv( this.quatExp( ks.* this.quatLog( this.quatProd(Qd,this.quatInv(this.Qgd)) ) ) );
            
            eqd = eqd'
            vRotd = vRotd'
            dvRotd = dvRotd'
            Qd = Qd'
            tau
            kt
            ks = ks'
            QQg = QQg'
            inv_exp_QdQgd = inv_exp_QdQgd'
            error('Stop');
            
            dvRot = kt^2*ks.*dvRotd - (this.a_z/tau)*(vRot-kt*ks.*vRotd) ...
                    -(this.a_z*this.b_z/tau^2) * this.quatLog ( this.quatProd(QQg, inv_exp_QdQgd)) + y_c;
            
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

    end
    
    methods (Static)
        
        function v_rot = quatLog(Q)

            n = Q(1);
            e = Q(2:4);
            norm_e = norm(e);

            if (norm_e > 1e-16)
                theta = 2*real(atan2(norm_e,n));
                v_rot = theta*e/norm_e;
            else
                v_rot = zeros(size(e));
            end

        end
        
        function Q = quatExp(v_rot)

            norm_v_rot = norm(v_rot);
            theta = norm_v_rot;

            if (norm_v_rot > 1e-16)
                Q(1) = cos(theta/2);
                Q(2:4) = sin(theta/2)*v_rot/norm_v_rot;
            else
                Q = [1 0 0 0]';
            end

        end
        
        function Q12 = quatProd(Q1, Q2)

            Q1 = Q1(:);
            Q2 = Q2(:);

            n1 = Q1(1);
            e1 = Q1(2:4);

            n2 = Q2(1);
            e2 = Q2(2:4);

            Q12 = zeros(4,1);
            Q12(1) = n1*n2 - e1'*e2;
            Q12(2:4) = n1*e2 + n2*e1 + cross(e1,e2);

        end
        
        function invQ = quatInv(Q)

            invQ = zeros(size(Q));

            invQ(1) = Q(1);
            invQ(2:4) = -Q(2:4);

        end

    end
end
