function []= waitforHUE(ard,M)

while(true)
        inp = char(readline(ard));
        disp(inp);
        if(inp(1) == M)
            disp("BREAK");
            break;
        end
end
