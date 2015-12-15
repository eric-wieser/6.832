function saveTraj(xtraj, fname)
ts = xtraj.tspan(1):1/60:xtraj.tspan(2);
ts = ts(1:end - 1);


data = xtraj.eval(ts);
fileID = fopen(fname,'w');
fwrite(fileID,data,'double');
fclose(fileID);

end

