# local_planner
Implementation of local planner using HUMP


Este planeador local utiliza o planeamento feito á priori pelo planeador HUMP. No caso de planear uma task utiliza os planeamentos de sucessivos movimentos.
Uma das capacidades que tem é online target acquisition, utilizando ou human_like movment ou trapezoidal move o hand effector até alvo.
O movimento cartesiano funciona sem problema, no entanto o end effector não se orienta apropriadamente com o alvo gerando comportamentos estranhos, imprevisiveis e descontrolados no braço.
