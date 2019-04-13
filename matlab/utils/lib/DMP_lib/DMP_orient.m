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
        
        Qgd
        Q0d
        tau_d
        
        eq_f
        vRot_f
        dvRot_f
        
        N_kernels % number of kernels (basis functions)

        a_z % parameter 'a_z' relating to the spring-damper system
        b_z % parameter 'b_z' relating to the spring-damper system

        can_clock_ptr % handle (pointer) to the canonical clock
        shape_attr_gating_ptr % pointer to gating function for the shape attractor

        zero_tol % tolerance value used to avoid divisions with very small numbers

        type % the DMP type

    end

    methods
        %% DMP constructor.
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
        function this = DMP_orient(a_z, b_z, N_kernels, can_clock_ptr, shape_attr_gating_ptr)

            this.zero_tol = 1e-30; %realmin;
            this.a_z = a_z;
            this.b_z = b_z;
            this.N_kernels = N_kernels;
            this.can_clock_ptr = can_clock_ptr;
            this.shape_attr_gating_ptr = shape_attr_gating_ptr;
            
            shapeAttrGatFun_ptr = @(x)getOutput(this.shape_attr_gating_ptr,x);
            for i=1:3
                this.eq_f{i} = WSoG(this.N_kernels(1,i), shapeAttrGatFun_ptr);
                this.vRot_f{i} = WSoG(this.N_kernels(2,i), shapeAttrGatFun_ptr);
                this.dvRot_f{i} = WSoG(this.N_kernels(3,i), shapeAttrGatFun_ptr);
            end

        end
        
        
        %% Trains the DMP_orient.
        %  @param[in] Time: Row vector with the timestamps of the training data points.
        %  @param[in] yd_data: Row vector with the desired potition.
        function [train_err] = train(this, train_method, Time, Qd_data, vRotd_data, dvRotd_data)

            n_data = length(Time);
            
            eqd = zeros(3, n_data);
            Qgd = Qd_data(:,end);
            this.Qgd = Qgd;
            this.Q0d = Qd_data(:,1);
            for j=1:n_data, eqd(:,j) = this.quatLog(this.quatProd(Qd_data(:,j),this.quatInv(Qgd))); end

            tau = Time(end);
            this.tau_d = tau;
            this.setTau(tau);
    
            x = this.phase(Time);
            
            if (train_method == DMP_orient.LWR), wsog_train_method = WSoG.LWR;
            elseif (train_method == DMP_orient.LS), wsog_train_method = WSoG.LS;  
            else, error('[DMP_orient::train]: Unsopported training method...');
            end
                
            train_err = zeros(3,3);
            for i=1:3
                train_err(1,i) = this.eq_f{i}.train(wsog_train_method, x, eqd(i,:));
                train_err(2,i) = this.vRot_f{i}.train(wsog_train_method, x, vRotd_data(i,:));
                train_err(3,i) = this.dvRot_f{i}.train(wsog_train_method, x, dvRotd_data(i,:));
            end

        end
        
        
        %% Returns the derivatives of the DMP states.
        %  @param[in] x: phase variable.
        %  @param[in] Q: Current orientation as unit quaternion.
        %  @param[in] vRot: Current rotational velocity.
        %  @param[in] Q0: initial orientation as unit quaternion.
        %  @param[in] Qg: Goal orientation as unit quaternion.
        %  @param[in] y_c: Coupling term.
        %  @param[out] dvRot: Rotational acceleration.
        %  @param[out] dx: Derivative of the phase variable.
        function [dvRot, dx] = getStatesDot(this, x, Q, vRot, Q0, Qg, y_c)

            if (nargin < 7), y_c=zeros(3,1); end

            dx = this.phaseDot(x);
            
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
            ks = this.quatLog( this.quatProd(Qg, this.quatInv(Q0) ) ) ./ this.quatLog( this.quatProd(this.Qgd, this.quatInv(this.Q0d) ) );
            
            QQg = this.quatProd(Q,this.quatInv(Qg));
            inv_exp_QdQgd = this.quatInv( this.quatExp( ks.* this.quatLog( this.quatProd(Qd,this.quatInv(this.Qgd)) ) ) );
            
            dvRot = kt^2*ks.*dvRotd - (this.a_z/tau)*(vRot-kt*ks.*vRotd) ...
                    -(this.a_z*this.b_z/tau^2) * this.quatLog ( this.quatProd(QQg, inv_exp_QdQgd)) + y_c;
            
        end
        

        %% Returns the time scale of the DMP_orient.
        %  @param[out] tau: The time scale of the this.
        function tau = getTau(this)

            tau = this.can_clock_ptr.getTau();

        end
        
        
        %% Sets the time scale of the DMP_orient.
        function tau = setTau(this, tau)

            this.can_clock_ptr.setTau(tau);

        end
        
        
        %% Returns the phase variable.
        %  @param[in] t: The time instant.
        %  @param[out] x: The phase variable for time 't'.
        function x = phase(this, t)
            
            x = this.can_clock_ptr.getPhase(t);

        end
        
        
        %% Returns the derivative of the phase variable.
        %  @param[in] x: The phase variable.
        %  @param[out] dx: The derivative of the phase variable.
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
