%% DMP_pos class
%  For encoding Cartesian Orientation.
%


classdef DMP_pos < handle
    
    properties (Constant)
        
          %% enum ARGUMENT
          
          %% enum TRAIN_METHOD
          LWR = 201;
          LS = 203;

    end
       
    properties
        
        dmp % 3x1 vector of DMP producing the desired trajectory for each x, y and z dimensions.

        can_clock_ptr % handle (pointer) to the canonical clock
        shape_attr_gating_ptr % pointer to gating function for the shape attractor
        
        dZ
        dY
        dx
    end

    methods
        %% DMP_pos constructor.
        %  @param[in] N_kernels: 3x3 matrix where each row contains the kernels for position,
        %                       velocity and acceleration and each row for x, y and z coordinate.
        %                       If 'N_kernels' is a column vector then every coordinate will have the
        %                       same number of kernels.
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
        %  @param[in] shape_attr_gating_ptr: Pointer to a DMP gating function object.
        %
        function this = DMP_pos(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr, dmp_type)
            
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
        
        
        %% Trains the DMP_pos.
        %  @param[in] train_method: The training method (see dmp_::TRAIN_METHOD enum).
        %  @param[in] Time: 1xN row vector with the timestamps of the training data points.
        %  @param[in] Pd_data: 3xN matrix with desired position at each column for each timestep.
        %  @param[in] dPd_data: 3xN matrix with desired velocity at each column for each timestep.
        %  @param[in] ddPd_data: 3xN matrix with desired acceleration at each column for each timestep.
        %  @param[in] train_error: Optinal pointer to return the training error as norm(F-Fd)/n_data.
        function train_err = train(this, train_method, Time, Pd_data, dPd_data, ddPd_data)

            tau = Time(end);
            this.setTau(tau);
            
            if (train_method == DMP_pos.LWR), dmp_train_method = DMP.LWR;
            elseif (train_method == DMP_pos.LS), dmp_train_method = DMP.LS;  
            else, error('[DMP_pos::train]: Unsopported training method...');
            end
                
            if (nargout > 0)
                train_err = zeros(3,1);
                for i=1:3
                    train_err(i) = this.dmp{i}.train(dmp_train_method, Time, Pd_data(i,:), dPd_data(i,:), ddPd_data(i,:));
                end
            else
                for i=1:3
                    this.dmp{i}.train(dmp_train_method, Time, Pd_data(i,:), dPd_data(i,:), ddPd_data(i,:));
                end
            end

        end
        
        %% Calculates the derivatives of the DMP states. The derivatives can then be
        %% retrieved with 'getDx', 'getDy' and 'getDz'.
        %  @param[in] x: phase variable.
        %  @param[in] Y: 'y' state of the DMP.
        %  @param[in] Z: 'z' state of the DMP.
        %  @param[in] Y0: Initial position.
        %  @param[in] Yg: Goal position.
        %  @param[in] Y_c: Coupling term for the dynamical equation of the 'y' state.
        %  @param[in] Z_c: Coupling term for the dynamical equation of the 'z' state.
        function calcStatesDot(this, x, Y, Z, Y0, Yg, Y_c, Z_c)
            
            if (nargin < 7), Y_c=zeros(3,1); end
            if (nargin < 8), Z_c=zeros(3,1); end
            
            for i=1:3, this.dmp{i}.calcStatesDot(x, Y(i), Z(i), Y0(i), Yg(i), Y_c(i), Z_c(i)); end
            
            this.dZ = zeros(3,1);
            this.dY = zeros(3,1);
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
        function dY = getDy(this), dY=this.dY; end
        function dZ = getDz(this), dZ=this.dZ; end
        
    end
    
end
