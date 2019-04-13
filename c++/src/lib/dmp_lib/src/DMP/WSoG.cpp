%% WSoG class
%  Weighted sum of Guassians.
%

classdef WSoG < handle
    
    properties (Constant)
          
          %% enum TRAIN_METHOD
          LWR = 201;
          LS = 203;

    end
       
    properties
        N_kernels % number of kernels (basis functions)

        shapeAttrGatingFun % pointer to gating function for the shape attractor
        
        w % N_kernelsx1 vector with the weights of the DMP
        c % N_kernelsx1 vector with the kernel centers of the DMP
        h % N_kernelsx1 vector with the kernel stds of the DMP

        zero_tol % tolerance value used to avoid divisions with very small numbers
        
        kernel_std_scaling % scales the std of the kernel function

        setVar % array with function pointers  
    end

    methods
        %% DMP constructor.
        %  @param[in] N_kernels: the number of kernels
        function this = WSoG(N_kernels, shapeAttrGatingFun, kernel_std_scaling) %, N_kernels, s_gat_ptr

            if (nargin < 3), kernel_std_scaling = 1.0; end
            
            this.N_kernels = N_kernels;
            this.shapeAttrGatingFun = shapeAttrGatingFun;
            this.kernel_std_scaling = kernel_std_scaling;
            
            this.zero_tol = 1e-30; %realmin;

            this.w = zeros(this.N_kernels,1);
            this.c = ((1:this.N_kernels)-1)'/(this.N_kernels-1);
            this.h = 1./(kernel_std_scaling*(this.c(2:end)-this.c(1:end-1))).^2;
            this.h = [this.h; this.h(end)];
            
        end

        
        function n_ker = getNumOfKernels(this)
            
            n_ker = length(this.w);
            
        end
        
        
        %% Trains the WSoG.
        %  @param[in] Time: Row vector with the timestamps of the training data points.
        %  @param[in] Fd_data: Row vector with the desired potition.
        function [train_error, F] = train(this, train_method, x, Fd)

            n_data = length(x);

            s = zeros(1, n_data);
            Psi = zeros(this.N_kernels, n_data);
            for i=1:n_data
                s(i) = this.shapeAttrGatingFun(x(i));
                Psi(:,i) = this.kernelFunction(x(i));
            end

            if (train_method == WSoG.LWR), this.w = LWR(Psi, s, Fd, this.zero_tol);
            elseif (train_method == WSoG.LS), this.w = leastSquares(Psi, s, Fd, this.zero_tol);
            else, error('[WSoG::train]: Unsopported training method...');
            end

            F = zeros(size(Fd));
            for i=1:size(F,2)
                F(i) = this.output(x(i));
            end

            train_error = norm(F-Fd)/length(F);

        end

        
        %% Returns a column vector with the values of the kernel functions of the WSoG.
        %  @param[in] x: phase variable
        %  @param[out] psi: column vector with the values of the kernel functions of the DMP
        function psi = kernelFunction(this,x)

            n = length(x);
            psi = zeros(this.N_kernels, n);

            for j=1:n
                psi(:,j) = exp(-this.h.*((x(j)-this.c).^2));
            end 

        end
         
        
        %% Returns the forcing term of the WSoG.
        %  @param[in] x: The phase variable.
        %  @param[out] f: The normalized weighted sum of Gaussians.
        function f = output(this,x)

            Psi = this.kernelFunction(x);
    
            f = this.shapeAttrGatingFun(x) * dot(Psi,this.w) / (sum(Psi)+this.zero_tol); % add 'zero_tol' to avoid numerical issues

        end

    end
end
