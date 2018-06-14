function J = kf_choi(inputs,srs)
%kf_choi Sends a 2x2 Q matrix to kf_choi.exe and uses it
%    to filter the requested data series. Computes the squares error
%    and returns the sum across 5 tests.

    J = 0;
    for i = 1:5
        seriesID = sprintf('%s_s%u',srs,i);
        command = sprintf('echo %0.4f %0.4f %0.1f %s | kf_choi.exe',...
            inputs(1),inputs(2),inputs(3),seriesID);
        [status,cmdout] = system(command);
        J = J + str2double(cmdout);
    end
    
end
