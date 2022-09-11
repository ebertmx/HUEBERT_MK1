function status = send2ard (ard, type, data)

if type == 'p'
    status = 0;
    disp("WRITING DATA_____________");
    S1=string(data(1));
    S2 = string(data(2));
    S3 = string(data(3));
    spd = string(data(4));
    
    s_output = 'p' +S1 + 'a' + S2 + 'b' + S3 + 'c'+spd+'s'
    write(ard, s_output,"string");
end


if type =='v'
    
    n = size(data);
    disp(string(n(1))+'L')
    s_output = 'v' + string(n(1)) + 'L';
    %write(ard,   s_output,'STRING');
    %inp = char(readline(ard))
    for i=1:n(1)
        S1=string(data(i,1));
        S2 = string(data(i,2));
        S3 = string(data(i,3));
        spd = string(data(i,4));
        s_output = s_output + S1 + 'a' + S2 + 'b' + S3 + 'c' + spd + 's';
    end
        write(ard, s_output,'STRING');
        %waitforHUE(ard,'E');
        %disp(inp);
end


if type =="func"
    write(ard, data,'STRING');
end
end

