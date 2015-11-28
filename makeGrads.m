function makeGrads(n)
  symdict = containers.Map;

  % constant symbols
  wheel = struct(...
    'pos', [0; 0],...
    'driveDir', [0; 0],...
    'slipDir', [0; 0],...
    'b', 1,...
    'r', 1);
  [wheel, symdict('wheel')] = symbolify(wheel, 'wheel');
  m = sym('m');
  I = sym('I');
  obj = HolonomicDrive([wheel], m, I);

  % symbols to differentiate wrt to
  [t, symdict('t')] = symbolify(0,'t');
  [q, symdict('q')] = symbolify(zeros(3,1), 'q');
  [qd, symdict('qd')] = symbolify(zeros(3,1), 'qd');
  [u, symdict('u')] = symbolify(0, 'u');
  sym_vector = [t; q; qd; u];

  % evaluate it:
  qddot = obj.sodynamics(t, q, qd, u);


  fp = fopen('oneWheelGradients.m', 'w');
  fprintf(fp, 'function [df] = oneWheelGradients(m, I, wheel, t, q, qd, u)\n')
  fprintf(fp, '%% Unpack variables\n');


  for varname = keys(symdict)
    varname = varname{1};
    varsyms = symdict(varname);
    for k = keys(varsyms)
      k = k{1};
      fprintf(fp, '%s = %s%s;\n', char(varsyms(k)), varname, k);
    end
    fprintf(fp, '\n');
  end

  qddot_dx = simplify(jacobian(qddot, sym_vector));

  fprintf(fp, '%% Conpute gradient\n');
  write_symbolic_matrix(fp, qddot_dx, 'df')
  fprintf(fp,'end\n');

  % close file
  fclose(fp);
end

function write_symbolic_matrix(fp,A,symbol)
  [m,n]=size(A);
  fprintf(fp,'%s = sparse(%d,%d);\n',symbol,m,n);
  for i=1:m, for j=1:n, if (A(i,j)~=0) fprintf(fp,'%s(%d,%d) = %s;\n',symbol,i,j,char(A(i,j))); end; end; end
end

