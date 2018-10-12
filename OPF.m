%% Constarint violation testing for IEEE14 bus system

function [ fxmin, x_min ] = OPF(bus,Pg,Qg,totalbus,totalbranch,all_busdata14, all_generatordata14,branch,generator)
% 
% global Ybus Yf Yt;
% global bus gen branch Yf Yt Ybus
%         [Ybus, Yf, Yt] = makeYbus(baseMVA, bus, branch);% construct Ybus
%         
%         [MVAbase,bus,gen,branch,success,et]=runpf; % run NR load flow
%          
%global V
%global Ybus



    Pmin=bus.RealPowerMin;
    Pmax=bus.RealPowerMax
    %Active_PowerGeneration=Pg' % active power output of all generators
    
    Qmin=bus.ReactivePowermin;
    Qmax=bus.ReactivePowermax;
    Reactive_PowerGeneration=Qg'; % active power output of all generators
   % Pgi=Pg';
    

nbus = totalbus; %  total number of buses (14)
bus.VoltageMagnitudeMin = all_busdata14(:,10);%min limit of voltage
bus.VoltageMagnitudeMax = all_busdata14(:,9); %max limit of voltage
VMAX=bus.VoltageMagnitudeMax; %min limit of voltage
VMIN=bus.VoltageMagnitudeMin; %max limit of voltage




% Determine the value of weight change
w_start = 1;   %Initial inertia weight's value
w_end = 0.20;       %Final inertia weight
w_varyfor = floor(0.7*100); 
w_now = w_start;
inertdec = (w_start-w_end)/w_varyfor; %Inertia weight's change per iteration
iter1=0;
no_particles=100;
iter1_max=50000;


% % no. of control variables=9
 Swarm=[unifrnd(Pmin(2,1),Pmax(2,1),no_particles,1),unifrnd(Pmin(3,1),Pmax(3,1),no_particles,1),unifrnd(Pmin(6,1),Pmax(6,1),no_particles,1),unifrnd(Pmin(8,1),Pmax(8,1),no_particles,1),unifrnd(0.90,1.10,no_particles,5)];
% 
%Vstep=20% of xmax-xmin= 0.2*(pi/2 -0)=0.3141
%VStep=[unifrnd(21.87,21.87,no_particles,1),unifrnd(15.34,15.34,no_particles,1), unifrnd(19.81,19.81,no_particles,1),unifrnd(15.96,15.96,no_particles,1),unifrnd(0.04,0.04,no_particles,5)]
xmin=[Pmin(2,1) Pmin(3,1) Pmin(6,1) Pmin(8,1) 0.9 0.9 0.9 0.9 0.9];
xmax=[Pmax(2,1) Pmax(3,1) Pmax(6,1) Pmax(8,1) 1.10 1.10 1.10 1.10 1.10];
xmin=repmat(xmin, no_particles , 1);
xmax=repmat(xmax, no_particles, 1);
Vmin = (-xmax + xmin)*0.2;
VStep = -Vmin;
% D= ;
% Swarm = zeros( no_particles, D );
% VStep = zeros( no_particles, D );
% for i = 1 : no_particles
%     Swarm( i, : ) = xmin + ( xmax - xmin ) .* rand( 1, D );
%     VStep( i, : ) = Vmin + ( Vmax - Vmin ) .* rand( 1, D )
% end




% vel_max::(max step length(velocity))
vel_max=(-(-xmax + xmin));    %[109.3543, 76.6980,99.0422,79.8249,0.2,0.2,0.2,0.2,0.2];
%vel_max=repmat(vel_max, no_particles, 1);

Pgggggg=zeros(no_particles,totalbus);
Qgggggg=zeros(no_particles,totalbus);

for z=1:no_particles % no of particles
   
%     all_generatordata14(1,2)=Swarm(z,1);
     all_generatordata14(2,2)=Swarm(z,1);
     all_generatordata14(3,2)=Swarm(z,2);
     all_generatordata14(4,2)=Swarm(z,3);
     all_generatordata14(5,2)=Swarm(z,4);
     all_busdata14(1,7)=Swarm(z,5);
     all_busdata14(2,7)=Swarm(z,6);
     all_busdata14(3,7)=Swarm(z,7);
     all_busdata14(6,7)=Swarm(z,8); 
     all_busdata14(8,7)=Swarm(z,9);
     
    

 [Ybus, Yf, Yt, Ys] = Ybus_1(totalbus,totalbranch,all_busdata14,branch,bus);
 LFNEWTON;
  Pgggggg(z,:)=Pg;
  Qgggggg(z,:)=Qg;
 
 %penalty value calculation for bus voltage violation
    Voltage_magnitude=Vm; % voltage mag. of each bus
   for count_1=1:nbus
        if Voltage_magnitude(count_1)>VMAX(count_1,:)
            penalty_volt(count_1)=10000*(Voltage_magnitude(count_1)-VMAX(count_1,:))^2;
        elseif Voltage_magnitude(count_1)<VMIN(count_1,:)
            penalty_volt(count_1)=10000*(VMIN(count_1,:)-Voltage_magnitude(count_1))^2;
        else
            penalty_volt(count_1)=0;
        end
   end
   penalty_volt=sum(penalty_volt);%summation of penalty for bus voltage violation
   
   

     Active_PowerGeneration=Pg';
    for count_2=1:nbus
        if  Active_PowerGeneration(count_2)>Pmax(count_2,:)
            penalty_active(count_2)=10000*((Active_PowerGeneration(count_2)-Pmax(count_2,:))^2);
        elseif Active_PowerGeneration(count_2)<Pmin(count_2,:)
            penalty_active(count_2)=10000*((Pmin(count_2,:)-Active_PowerGeneration(count_2))^2);
        else
            penalty_active(count_2)=0;
        end
    end
    penalty_active=sum(penalty_active);  %summation of penalty for Generator Real Power Bounds Violtaion
    
    

  % penalty value calculation for Generator Reactive Power Bound violation 
    Reactive_PowerGeneration=Qg';   % active power output of all generators
    for count_3=1:nbus
        if  Reactive_PowerGeneration(count_3)>Qmax(count_3,:)
            penalty_reactive(count_3)=10000*((Reactive_PowerGeneration(count_3)-Qmax(count_3,:))^2);
        elseif Reactive_PowerGeneration(count_3)<Qmin(count_3,:)
            penalty_reactive(count_3)=10000*((Qmin(count_3,:)-Reactive_PowerGeneration(count_3))^2);
        else
            penalty_reactive(count_3)=0;
        end
    end
    penalty_reactive=sum(penalty_reactive);   %summation of penalty for Generator Real Power Bounds Violtaion


    
     APF=Pij(:,1).^2;
     RPF=Qij(:,1).^2;
     SPF=sqrt(APF+RPF); %actual values of apparent powers "From bus injection" 
     APF1=Pji(:,1).^2;
     RPF1=Qji(:,1).^2;
     SPF1=sqrt(APF1+RPF1); %actual values of apparent powers "To bus injection" 
     jjj=[];
     nbr=totalbranch;

%    %penalty value calculation for line flow violations
  for count_1=1:nbr; % total number of lines (20)
      if SPF(count_1)>branch.PowerMagnitudeMax(count_1,:)
          kkk(count_1)=10000*((SPF(count_1)-branch.PowerMagnitudeMax(count_1,:))^2); % if limit violates,penalty is imposed
      else
          kkk(count_1)=0; %if no violation, penalty is zero
      end
  end
penalty_line=sum(kkk);  % summation of all penalty of line flow violations%



  for count_1=1:nbr; % total number of lines (20)
      if SPF1(count_1)>branch.PowerMagnitudeMax(count_1,:)
          kkk(count_1)=10000*((SPF1(count_1)-branch.PowerMagnitudeMax(count_1,:))^2); % if limit violates,penalty is imposed
      else
          kkk(count_1)=0; %if no violation, penalty is zero
      end
  end
penalty_line1=sum(kkk);


Total_penalty= penalty_volt+ penalty_active + penalty_reactive + penalty_line + penalty_line1;
%Active_PowerGeneration= Active_PowerGeneration';
%% Calculation of Objective Function
 Pgi=Pg';
for p=1:nbus;
    
generator.a1= zeros(totalbus,1);
generator.a1(all_generatordata14(:,1),1) = generator.RealPowerCostCoefficient(:,1);
a1=generator.a1;

generator.b1= zeros(totalbus,1);
generator.b1(all_generatordata14(:,1),1) = generator.RealPowerCostCoefficient(:,2);
b1=generator.b1;

generator.c1= zeros(totalbus,1);
generator.c1(all_generatordata14(:,1),1) = generator.RealPowerCostCoefficient(:,3);
c1=generator.c1;


 F1(p,1)=(Pgi(p,1).*Pgi(p,1))*c1(p,1)+Pgi(p,1)*b1(p,1)+(a1(p,1));

end

% global sum_Fuelcost
% sum_Fuelcost=(sum(F1));
global Total_penalty
fun_Swarm=(sum(F1))+(Total_penalty);
% Objective function= sum of active power losses of the transmission lines
fSwarm(z,:)= sum(F1)+Total_penalty;
end 

% Initializing the Best positions matrix and
% the corresponding function values
PBest = Swarm; 
fPBest = fSwarm;
% Finding best particle in initial population
[fGBest, g] = min(fSwarm);
lastbpf = fGBest;
Best = Swarm(g,:); %Used to keep track of the Best particle ever
fBest = fGBest;
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%                  THE  PSO  LOOP                          %%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while( iter1 < iter1_max )% no of iterations
 
    iter1 = iter1+1
  
    % Update the value of the inertia weight w
    if (iter1<=w_varyfor) & (iter1 > 1)
        w_now = w_now - inertdec; %Change inertia weight
    end 
   
    % The PLAIN PSO %
    % Set GBest
    AGbest = repmat(Swarm(g,:), no_particles, 1); %A = GBest. repmat(X, m, n) repeats the matrix X in m rows by n columns, 50=no. of particles       
    % Generate Random Numbers
    R1 = rand(no_particles,9); % 50=No. of particles, 9=dimension of the problem(size of swarm=no. of variables)
    R2 = rand(no_particles,9);
    
    % Calculate Velocity
    VStep = w_now*VStep + 2.1*R1.*(PBest-Swarm) + 2.0*R2.*(AGbest-Swarm);% c1=2.1, c2=2
    
   %check max. velocity limit violation 
changemax_vel=VStep>vel_max;
VStep(changemax_vel)=vel_max(changemax_vel);
%check -max.(i.e. min.) velocity limit violation 
changemin_vel=VStep<(-vel_max);
VStep(changemin_vel)=-vel_max(changemin_vel);

    
    % ::UPDATE POSITIONS OF PARTICLES::
    Swarm = Swarm + 0.729 * VStep;  % Evaluate new Swarm, Chi=0.729
    
   % check limits of x.(min and max limits)
    changemax=Swarm>xmax;
    Swarm(changemax)=xmax(changemax);
    changemin=Swarm<xmin;
    Swarm(changemin)=xmin(changemin);
    
    Pggg=zeros(no_particles,totalbus);
    Qggg=zeros(no_particles,totalbus);
    
  for j=1:no_particles % no of particles
   
     %all_generatordata14(1,2)=Swarm(j,1);
     all_generatordata14(2,2)=Swarm(j,1);
     all_generatordata14(3,2)=Swarm(j,2);
     all_generatordata14(4,2)=Swarm(j,3);
     all_generatordata14(5,2)=Swarm(j,4);
     all_busdata14(1,7)=Swarm(j,5);
     all_busdata14(2,7)=Swarm(j,6);
     all_busdata14(3,7)=Swarm(j,7);
     all_busdata14(6,7)=Swarm(j,8); 
     all_busdata14(8,7)=Swarm(j,9);
     
  %[Ybus, Yf, Yt, Ys] = Ybus_1(totalbus,totalbranch,all_busdata14,branch,bus);
    LFNEWTON;
    Pggg(j,:)=Pg;
    Qggg(j,:)=Qg;
%    
%    for jjjj=1:no_particles
%     Pggg(jjjj,14)=Pg(:,14);
%    end
 
 %penalty value calculation for bus voltage violation
    Voltage_magnitude=Vm; % voltage mag. of each bus
   for count_1=1:nbus
        if Voltage_magnitude(count_1)>VMAX(count_1,:)
            penalty_volt(count_1)=10000*(Voltage_magnitude(count_1)-VMAX(count_1,:))^2;
        elseif Voltage_magnitude(count_1)<VMIN(count_1,:)
            penalty_volt(count_1)=10000*(VMIN(count_1,:)-Voltage_magnitude(count_1))^2;
        else
            penalty_volt(count_1)=0;
        end
   end
   penalty_volt=sum(penalty_volt);%summation of penalty for bus voltage violation

     
   Active_PowerGeneration=Pg';
    for count_2=1:nbus
        if  Active_PowerGeneration(count_2)>Pmax(count_2,:)
            penalty_active(count_2)=10000*((Active_PowerGeneration(count_2)-Pmax(count_2,:))^2);
        elseif Active_PowerGeneration(count_2)<Pmin(count_2,:)
            penalty_active(count_2)=10000*((Pmin(count_2,:)-Active_PowerGeneration(count_2))^2);
        else
            penalty_active(count_2)=0;
        end
    end
    penalty_active=sum(penalty_active);  %summation of penalty for Generator Real Power Bounds Violtaion
    
    

  % penalty value calculation for Generator Reactive Power Bound violation 
    Reactive_PowerGeneration=Qg';   % active power output of all generators
    for count_3=1:nbus
        if  Reactive_PowerGeneration(count_3)>Qmax(count_3,:)
            penalty_reactive(count_3)=10000*((Reactive_PowerGeneration(count_3)-Qmax(count_3,:))^2);
        elseif Reactive_PowerGeneration(count_3)<Qmin(count_3,:)
            penalty_reactive(count_3)=10000*((Qmin(count_3,:)-Reactive_PowerGeneration(count_3))^2);
        else
            penalty_reactive(count_3)=0;
        end
    end
    penalty_reactive=sum(penalty_reactive);   %summation of penalty for Generator Real Power Bounds Violtaion


     APF=Pij(:,1).^2;
     RPF=Qij(:,1).^2;
     SPF=sqrt(APF+RPF); %actual values of apparent powers "From bus injection" 
     APF1=Pji(:,1).^2;
     RPF1=Qji(:,1).^2;
     SPF1=sqrt(APF1+RPF1); %actual values of apparent powers "To bus injection" 
     jjj=[];
     nbr=totalbranch;

%    %penalty value calculation for line flow violations
  for count_1=1:nbr; % total number of lines (20)
      if SPF(count_1)>branch.PowerMagnitudeMax(count_1,:)
          kkk(count_1)=10000*((SPF(count_1)-branch.PowerMagnitudeMax(count_1,:))^2); % if limit violates,penalty is imposed
      else
          kkk(count_1)=0; %if no violation, penalty is zero
      end
  end
penalty_line=sum(kkk);  % summation of all penalty of line flow violations
%     

  for count_1=1:nbr; % total number of lines (20)
      if SPF1(count_1)>branch.PowerMagnitudeMax(count_1,:)
          kkk(count_1)=10000*((SPF1(count_1)-branch.PowerMagnitudeMax(count_1,:))^2); % if limit violates,penalty is imposed
      else
          kkk(count_1)=0; %if no violation, penalty is zero
      end
  end
penalty_line1=sum(kkk);

Total_penalty= penalty_volt+ penalty_active + penalty_reactive + penalty_line + penalty_line1;
%Active_PowerGeneration= Active_PowerGeneration';
%% Calculation of Objective Function
for p=1:nbus;
    
% generator.a1= zeros(totalbus,1);
% generator.a1(all_generatordata14(:,1),1) = generator.RealPowerCostCoefficient(:,1);
% a1=generator.a1;
% 
% generator.b1= zeros(totalbus,1);
% generator.b1(all_generatordata14(:,1),1) = generator.RealPowerCostCoefficient(:,2);
% b1=generator.b1;
% 
% generator.c1= zeros(totalbus,1);
% generator.c1(all_generatordata14(:,1),1) = generator.RealPowerCostCoefficient(:,3);
% c1=generator.c1;

 F1(p,1)=(Pgi(p,1).*Pgi(p,1))*c1(p,1)+Pgi(p,1)*b1(p,1)+(a1(p,1));
end
global penalty_function
fun_Swarm=sum(F1)+Total_penalty;
% Objective function= sum of active power losses of the transmission lines
fSwarm(j,:)=sum(F1)+Total_penalty;



% zzz=Swarm
% Pg
% Pggg=zeros(no_particles,14)
% for jjjj=1:no_particles
%     Pggg(jjjj,14)=Pg(:,14);
% end

% Pgiii=zeros(1,totalbus)
% 
% Pgiii(all_generatordata14(:,1),1) = Pg(:,1);
% Pgii(j)=Pgiii


  end

    % Updating the best position for each particle
    changeRows = fSwarm < fPBest;                      % latest fswarm-previous fswarm
    fPBest(find(changeRows)) = fSwarm(find(changeRows)); %fitness value
    PBest(find(changeRows), :) = Swarm(find(changeRows), :); %position
    lastbpart = PBest(g, :);
    % Updating index g
    [fGBest, g] = min(fPBest);
    
    
    %Update Best. Only if fitness has improved.
    if fGBest < lastbpf
        [fBest, b] = min(fPBest);
        fbest(iter1) = fBest; 
        fBest;
        Best = PBest(b,:);
   

    end
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                  END  OF PSO  LOOP                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[fxmin, b] = min(fPBest)
x_min = PBest(b, :)
Psopg=Pggg
Basicpg=Pgggggg;
Psoqg=Qggg;
Basicqg=Qgggggg;



  
  
  
 
  
