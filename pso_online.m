%based in modified script from M. N. Alam, "Particle Swarm Optimization: 
% Algorithm and its Codes in MATLAB," ResearchGate (2016) 1-10, 2016. doi:
%http://dx.doi.org/10.13140/RG.2.1.4985.3206
%simplified script. Evaluate objective function only one time each step
%time. Compatible with Simulink Coder and OPAL-RT

function [y,KS1G1,num1,den1,num2,den2,ctr]= pso_online(OF_in, ...
   LB_, UB_, popSize_, w_, c_, KS1G1_, T1G1_, T2G1_, T3G1_,T4G1_,enable, tol_ref_)
%Parameters:
    % Caracter "_" means that is a parameter defined in block mask
    % LB_: Vector. Particle positions lower bound. (PSS parameters)
    % UB_: Vector. Particle positions upper bound. 
    % popSize_: PSO population size.
    % w_: Inertia constant.
    % c_: Learning factor (Cheng 2007).
    % KS1G1_: Gain KS1.
    % T1G1_: Lead/lag time constant T1.
    % T2G1_: Lead/lag time constant T2.    
    % T3G1_: Lead/lag time constant T3.
    % T4G1_: Lead/lag time constant T4.


rng default

%lower and upper bounds for particle pos values
LB=LB_; %lower bounds of variables
UB=UB_; %upper bounds of variables
%variable initialization (preallocation)
popSize=popSize_; % population size
numVar=length(LB); % number of variables
w = w_;
c = c_;
%compares OF value with a tolerance
tol_ref = tol_ref_ ;

% persistent variables
persistent x0
persistent f0
persistent f
persistent particleNumber
persistent x
persistent v
persistent pbest %best position
persistent movNumber %actual movement number. Only a counter
persistent pb %value of particle best for output
persistent counter



if isempty(particleNumber)
    
    f0 = 1000; %high value for the first result of the objective function
    f = zeros(popSize,1);
    x0 = zeros(popSize,numVar);
    x = zeros(popSize,numVar);
    v = zeros(popSize,numVar);
    pbest =  zeros(popSize,numVar);
    pb = [KS1G1_,T1G1_,T2G1_,T3G1_,T4G1_];
    particleNumber=0;
    movNumber = 1;
    counter=0;
end
counter = counter+1;

if enable && OF_in >= tol_ref 
    %counter = counter+1 ;
    
    
    if particleNumber == 0 
        
        % pso main program------------------------------------------------start
        % pso initialization----------------------------------------------start
        %before first run, only initialize this variables once
        f0 = OF_in;
        for i=1:popSize
            
            for j=1:numVar
            
                x0(i,j)=LB(j)+rand()*(UB(j)-LB(j));
            
            end
        
        end
        x=x0; % initial population
        v=0.1*x0; % initial velocity
        pbest = x0;
        particleNumber = particleNumber +1;
        y =[KS1G1_,T1G1_,T2G1_,T3G1_,T4G1_];
        KS1G1 = y(1);
        num1 = [1 y(2)];
        den1 = y(3);
        num2 = [1 y(4)];
        den2 = y(5);
        ctr = counter;

        return
    
    elseif particleNumber > 0 && particleNumber<=popSize 
    
        % ----------------- evaluating fitness----------------------
        %evaluate only one time for each time step
        %objective function
        f(particleNumber,1) = OF_in;
        %check if acceptable results has been reached
%         if OF_in < tol_ref
%         
%             y = x(particleNumber,:);
%             KS1G1 = y(1);
%             num1 = [1 y(2)];
%             den1 = y(3);
%             num2 = [1 y(4)];
%             den2 = y(5);
%             %updating particle best (pbest)
%             if f(particleNumber,1)<f0
%             
%                 pbest(particleNumber,:)=x(particleNumber,:);
%                 f0=f(particleNumber,1);
%                 pb = x(particleNumber,:);
%             
%             end
%             return
%         
%         end
        % --------- updating pbest and fitness asynchronous ---------------        
        %updating particle best (pbest)
        if f(particleNumber,1)<f0
            
            pbest(particleNumber,:)=x(particleNumber,:);
            f0=f(particleNumber,1);
            pb = x(particleNumber,:);

        end
        % --------------- Next particle movement --------------------------        
        particleNumber = particleNumber +1;       
        if particleNumber == popSize+1
            
            y = pb;
            KS1G1 = y(1);
            num1 = [1 y(2)];
            den1 = y(3);
            num2 = [1 y(4)]; 
            den2 = y(5);
       ctr = counter;

        else
            
            y = x(particleNumber,:);
            KS1G1 = y(1);
            num1 = [1 y(2)];
            den1 = y(3);
            num2 = [1 y(4)];
            den2 = y(5);
            ctr= counter;
        
        end
        return
    elseif  particleNumber == popSize +1
        %f0 = OF_in;
        particleNumber = 1;           
        movNumber = movNumber +1;
        % -------- pso velocities update ----------------------
        for i = 1:popSize
        
            for j=1:numVar                

                %cheng 2007 ,fixed w = 0.725
                v(i,j)=w*v(i,j)+c*rand()*(pbest(i,j)-x(i,j));
            
            end
        
        end

        % ----------------- pso position update---------------
        %x = x+v;
        x = pb+v;
        % -----------handling boundary violations--------------------
        for i=1:popSize
            
            for j=1:numVar
                
                if x(i,j)<LB(j)
                    
                    x(i,j)=LB(j);
                
                elseif x(i,j)>UB(j)
                    
                    x(i,j)=UB(j);
                
                end
             
            end
         
        end
        y = x(particleNumber,:);
        KS1G1 = y(1);
        num1 = [1 y(2)];
        den1 = y(3);
        num2 = [1 y(4)];
        den2 = y(5);
        ctr = counter;

        return
    end 
    % pso algorithm-----------------------------------------------------end

else % enable

    %updating output
    y = pb;
    KS1G1 = y(1);
    num1 = [1 y(2)];
    den1 = y(3);
    num2 = [1 y(4)];
    den2 = y(5);
    ctr = counter;

    return

end 
% As of now, the following lines never are executed
% Placeholder to avoid "Output argument 'y' is not assigned on some
% execution paths." error.
fprintf('WARNING: Placeholder lines executed.')
y = [KS1G1_,T1G1_,T2G1_,T3G1_,T4G1_];
KS1G1 = y(1);
num1 = [1 y(2)];
den1 = y(3);
num2 = [1 y(4)];
den2 = y(5);
ctr = counter;
return