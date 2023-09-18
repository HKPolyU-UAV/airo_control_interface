%
% Copyright (c) The acados authors.
%
% This file is part of acados.
%
% The 2-Clause BSD License
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright notice,
% this list of conditions and the following disclaimer in the documentation
% and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.;

%


SOURCES = { ...
            'quadrotor_tailsitter_hybrid_reduced_model/quadrotor_tailsitter_hybrid_reduced_expl_ode_fun.c', ...
            'quadrotor_tailsitter_hybrid_reduced_model/quadrotor_tailsitter_hybrid_reduced_expl_vde_forw.c',...
            'acados_solver_sfunction_quadrotor_tailsitter_hybrid_reduced.c', ...
            'acados_solver_quadrotor_tailsitter_hybrid_reduced.c'
          };

INC_PATH = '/home/rocky/acados/examples/acados_matlab_octave/getting_started/../../../include';

INCS = {['-I', fullfile(INC_PATH, 'blasfeo', 'include')], ...
        ['-I', fullfile(INC_PATH, 'hpipm', 'include')], ...
        ['-I', fullfile(INC_PATH, 'acados')], ...
        ['-I', fullfile(INC_PATH)]};



CFLAGS = 'CFLAGS=$CFLAGS';
LDFLAGS = 'LDFLAGS=$LDFLAGS';
COMPFLAGS = 'COMPFLAGS=$COMPFLAGS';
COMPDEFINES = 'COMPDEFINES=$COMPDEFINES';



LIB_PATH = ['-L', fullfile('/home/rocky/acados/examples/acados_matlab_octave/getting_started/../../../lib')];

LIBS = {'-lacados', '-lhpipm', '-lblasfeo'};

% acados linking libraries and flags
    
    


try
    %     mex('-v', '-O', CFLAGS, LDFLAGS, COMPFLAGS, COMPDEFINES, INCS{:}, ...
    mex('-O', CFLAGS, LDFLAGS, COMPFLAGS, COMPDEFINES, INCS{:}, ...
            LIB_PATH, LIBS{:}, SOURCES{:}, ...
            '-output', 'acados_solver_sfunction_quadrotor_tailsitter_hybrid_reduced' );
catch exception
    disp('make_sfun failed with the following exception:')
    disp(exception);
    disp('Try adding -v to the mex command above to get more information.')
    keyboard
end

fprintf( [ '\n\nSuccessfully created sfunction:\nacados_solver_sfunction_quadrotor_tailsitter_hybrid_reduced', '.', ...
    eval('mexext')] );


%% print note on usage of s-function, and create I/O port names vectors
fprintf('\n\nNote: Usage of Sfunction is as follows:\n')
input_note = 'Inputs are:\n';
i_in = 1;

global sfun_input_names
sfun_input_names = {};
input_note = strcat(input_note, num2str(i_in), ') lbx_0 - lower bound on x for stage 0,',...
                    ' size [8]\n ');
sfun_input_names = [sfun_input_names; 'lbx_0 [8]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') ubx_0 - upper bound on x for stage 0,',...
                    ' size [8]\n ');
sfun_input_names = [sfun_input_names; 'ubx_0 [8]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') y_ref_0, size [11]\n ');
sfun_input_names = [sfun_input_names; 'y_ref_0 [11]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') y_ref - concatenated for shooting nodes 1 to N-1,',...
                    ' size [209]\n ');
sfun_input_names = [sfun_input_names; 'y_ref [209]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') y_ref_e, size [8]\n ');
sfun_input_names = [sfun_input_names; 'y_ref_e [8]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') lbu for shooting nodes 0 to N-1, size [60]\n ');
sfun_input_names = [sfun_input_names; 'lbu [60]'];
i_in = i_in + 1;
input_note = strcat(input_note, num2str(i_in), ') ubu for shooting nodes 0 to N-1, size [60]\n ');
sfun_input_names = [sfun_input_names; 'ubu [60]'];
i_in = i_in + 1;

fprintf(input_note)

disp(' ')

output_note = 'Outputs are:\n';
i_out = 0;

global sfun_output_names
sfun_output_names = {};
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') u0, control input at node 0, size [3]\n ');
sfun_output_names = [sfun_output_names; 'u0 [3]'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') acados solver status (0 = SUCCESS)\n ');
sfun_output_names = [sfun_output_names; 'solver_status'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') KKT residual\n ');
sfun_output_names = [sfun_output_names; 'KKT_residual'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') x1, state at node 1\n ');
sfun_output_names = [sfun_output_names; 'x1'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') CPU time\n ');
sfun_output_names = [sfun_output_names; 'CPU_time'];
i_out = i_out + 1;
output_note = strcat(output_note, num2str(i_out), ') SQP iterations\n ');
sfun_output_names = [sfun_output_names; 'sqp_iter'];

fprintf(output_note)

% The mask drawing command is:
% ---
% global sfun_input_names sfun_output_names
% for i = 1:length(sfun_input_names)
% 	port_label('input', i, sfun_input_names{i})
% end
% for i = 1:length(sfun_output_names)
% 	port_label('output', i, sfun_output_names{i})
% end
% ---
% It can be used by copying it in sfunction/Mask/Edit mask/Icon drawing commands
%   (you can access it wirth ctrl+M on the s-function)