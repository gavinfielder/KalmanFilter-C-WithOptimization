function J = kf_choi(Q,srs)
%kf_choi Sends a 2x2 Q matrix to kf_choi.exe and uses it
%    to filter the requested data series. Computes the squares error
%    and returns the sum across 5 tests.

    J = 0;
    for i = 1:5
        seriesID = sprintf('%s_s%u',srs,i);
        command = sprintf('echo %0.4f %0.4f %0.4f %0.4f %s | kf_choi.exe',...
            Q(1,1),Q(1,2),Q(2,1),Q(2,2),seriesID);
        [status,cmdout] = system(command);
        J = J + str2double(cmdout);
    end
    
end
