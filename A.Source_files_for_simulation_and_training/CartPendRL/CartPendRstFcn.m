function simoutput = rstfcn(siminput)
    persistent toggle
    if (isempty(toggle))
        toggle = 0;
    end
    toggle = ~toggle;
    simoutput = setVariable(siminput, 'x0', [rand - 0.5; toggle*pi; 0; 0], 'Workspace', 'CartPendSim');
end