function []= waitforHUE(ard,M)

while(true)
        inp = char(readline(ard));
        disp(inp);
        if(inp == M)
            disp("BREAK");
            break;
        end
end
