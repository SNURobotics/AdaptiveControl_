function Time = ComputeDeadZone(state_parameter,Euclidean_projection,NaturalAdaptation)
if(NaturalAdaptation)
    Time = [];
    return;
end

nLink = size(state_parameter,2)/10;
threshold = 1e-6;
Time = zeros(size(state_parameter,1),1);

if(Euclidean_projection)
    % no change
    for n=1:size(state_parameter,1)
        for link = 1:nLink
            state_parameter_target = state_parameter(n, 10 * (link-1)+1:10 * link)';
            e = eig(G2S(p2G(state_parameter_target)));
            minEigenvalue = e(1);
            if((0 < minEigenvalue) && (minEigenvalue < 1e-3))
                Time(n) = n;
                break
            end
        end
    end
else
    % less than zero
    for n=1:size(state_parameter,1)
        for link = 1:nLink
            state_parameter_target = state_parameter(n, 10 * (link-1)+1:10 * link)';
            e = eig(G2S(p2G(state_parameter_target)));
            minEigenvalue = e(1);
            if(minEigenvalue < 0)
                Time(n) = n;
                break
            end
        end
    end
end

% remove zeros
Time(Time==0) = [];

end