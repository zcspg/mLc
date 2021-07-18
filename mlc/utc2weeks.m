
function [weekNo, secondOfweek] = utc2weeks(year, month, day, hour, minute, second)

    DayofYear = 0;
    DayofMonth = 0;

    for i = 1980 : year-1
        if((mod(i,4) == 0 && mod(i,100) ~= 0) || mod(i,400)  == 0)
            DayofYear = DayofYear + 366;
        else
            DayofYear = DayofYear + 365;
        end
    end
    for i = 1 :  month-1
        if (i == 1 || i == 3 || i == 5 || i == 7 || i == 8 || i == 10 || i ==12)
            DayofMonth = DayofMonth + 31;
        elseif (i == 4 || i == 6 || i == 9 || i == 11)
            DayofMonth = DayofMonth + 30;
        else
            if ((mod(year,4) == 0 && mod(year,100) ~= 0) || mod(year,400) == 0)
                DayofMonth = DayofMonth + 29;
            else
                DayofMonth = DayofMonth + 28;
            end
        end
    end

    Day = DayofMonth + day + DayofYear-6;
    weekNo = Day / 7;
    secondOfweek = mod(Day,7) * 86400 + hour * 3600 + minute * 60 + second + 18;
 