1. COMPONENT:
Information include:
a) all Pins
b) voltage/current boundary conditions for pins
c) dependent pins (used in connection)
d) all optimization variables (pins + additional variables e.g. Velocity/Torque
   of Motor; Duty cycles of Micro-Controller)
e) all optimization variables types (CONTINUOUS)
f) boundary conditions for optimization variables
g) linear model matrix 
h) lp equation relations (LESS, EQUAL, GREATER)
i) lp equation names (constraint on input/output/...)  
j) lp equation <pin, index> model index map (used in solver)  
k) additional linear constraints (e.g. Vin > Vout + 1V  for voltage regulator)

All components can be initialized through function: intializeAllXXX() and stored
in <string, Component> map for quick access.

2. CIRCUIT
Circuit takes two inputs: components and their pin connections.
Flow of solving:
a) add all new components variables into solver
b) specify objective (e.g. max/min all used pins voltage)
c) add used pin related lp equations into model (access by model index map)
d) add additional linear constraints of new components
e) add equality constraints specified by pin connections
f) optimize
g) check optimality 
h) if optimal: iterate through different points until optimal at linearized 
   point; if not: throw error
i) update used pins voltage lb/ub

Note that every model has to be solved twice to get compatible voltage ranges
(v_min, v_max) and for a practical solving reason that when objective is: min 
all used pin voltages, the optimial point could lead to furtur solver failure.

3. MATCHING 
Break down of matching process:
a) multiple pairs of components matching 
b) single pair of components matching
c) group of pins matching 
d) pin-to-pin matching

a) multiple pairs of components matching: the idea here is to priorize pairs. 
Pairs include actuators/sensors will be the first to be connected; if actuator/
sensor pairs are run out, pairs with least occurance number components are 
matched then. Loop until are pairs are connected.

b) single pair matching: left component is output, right component is input. 
Compatible groups of pins are defined. (e.g. left power out -- right power in; 
left function out -- right function in). All left and right components groups 
of pins are represents and based on the type of components, two mask operations
are performed to get compatible groups of pins.

c) iterate through two group of pins; Add additional connections on duty cycles 
when micro controller is involved.

d) a pair of pins has to satisfy 6 condtions to be connected:
   i} input pin must be usable
   ii) both of pins must be active
   iii) pins voltage ranges intersect
   iv) output pin has enough current for input pin (not added right now)
   v) physical type must be the same
   vi) function types must be convertible
   
Note that several maps are defined to facilitate matching process: 
i) <component, component> connetion map defines which two components can be 
connected;
ii) <componentpair, mask_type> connection_mask_map defines if power pins or function
pins are going to be connected;
iii) <functioon type, function type> compatible_type_map defines which two pins can
be connected 
iv) All compatible groups of pins are defined in connection_code_vec;

4. INFERENCE
inference is a recursive process, it follows: 
a) run maximum node connections 
e.g. c11 c12 c13 c14
     c21 c22 c23
	 c31 c32
   Imagine we have c31, c32 components, the maximum connection process will try 
   to connect (c31, c32) to (c21, c22, c23) and then connect(c31, c32) to {c11, c12, 
   c13, c14} and finally connected (c31, c32) themselves. The reason to connect 
   previous layer of components is to get dependent pins of (c31, c32). The output
   is pin connections of all these components. Parent-children relation will be 
   establish if a connection exists between two components.
b) solving subsytem
c) compute electrical properities of (c31, c32) for inference needs. Here, they 
are power input currents/function input currents/power input voltage ranges/
function input voltage ranges. Power input current is computed by adding 
previous components' currents.
d) infer next level types of components. Here the inference logic is defined 
in <component, component_vec> map. If a component has at least one descedent 
for every power/function input requirements, then this component will not infer 
any components.
e) run minimum clique cover to group intersectd voltage ranges and currents 
together.
g) infer versions of components. For every type of component, voltage and 
current are used to get compatible next level components. Components satisfy
i >= i_prev, v >= v_prev are considered as valid options. Whether power voltage/
current or function power/current is used is determined by a <component_types, 
input_type> map.
h) infer numbers of components. For ever version of component, number is 
determined by ceil(ouput_metric/input_metric). The metric could be number of 
pins for functiion requirements or current value for power requirements. This
metric is also decided by <component_types, metric_type> map 
i) create new nodes 







