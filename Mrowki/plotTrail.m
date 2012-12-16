function plotTrail(array)

max=length(array.trail);
for n=1:max
    x(n)=array.trail{n}(1);
    y(n)=array.trail{n}(2);
end


plot(x,y,'color','white','lineWidth',2)
