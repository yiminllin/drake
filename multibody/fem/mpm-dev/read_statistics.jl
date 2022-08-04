using Plots
ENV["GKSwstype"] = "nul"

t  = []
s  = []
m  = []
Ek = []
Es = []
Ep = []
E  = []
mv = []
ma = []
pn = []
pt = []
gn = []
gt = []

gr(x_lim=[0,2],legend=false,label=false)
anim = Animation()

plotw = 10
hidew = 4



# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/rotating-sphere/outputs-CFL.8"
# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/pingpong/outputs-withoutg"
# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/slide-under-slope/outputs-mu.0"
# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/slide-under-slope/outputs-mu.27"
# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/shaking-sphere/outputs-mu.1"

# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/slide-under-slope/outputs-mu.268"
# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/slide-under-slope/outputs-mu.25"
# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/slide-under-slope/outputs-mu.5"
# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/shaking-sphere/outputs-mu1000"
# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/shaking-sphere/outputs-mu.01"
# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/shaking-sphere/outputs-mu.1-v5"
# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/stick-slip-transition/outputs-mu0"
# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/stick-slip-transition/outputs-mu.5"
# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/stick-slip-transition/outputs-mu1"

# OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/rotating-sphere/outputs-CFL.2"
open("$(OUTPUT_NAME)/statistics.dat") do f
    line = 0

    while !eof(f)
        line += 13
        append!(t, parse(Float64, readline(f)))
        append!(s, parse(Float64, readline(f)))
        append!(m, parse(Float64, readline(f)))
        append!(Ek, parse(Float64, readline(f)))
        append!(Es, parse(Float64, readline(f)))
        append!(Ep, parse(Float64, readline(f)))
        append!(E, parse(Float64, readline(f)))
        append!(mv, parse(Float64, readline(f)))
        append!(ma, parse(Float64, readline(f)))
        append!(pn, parse(Float64, readline(f)))
        append!(pt, parse(Float64, readline(f)))
        append!(gn, parse(Float64, readline(f)))
        append!(gt, parse(Float64, readline(f)))
    end
end

# anim = Animation()
# for i = 1:length(t)-1
#     p = plot(t, E, lw=plotw)
#     plot!(p, t[i:end], E[i:end],linecolor=:white,lw=hidew,xtickfont = font(20),ytickfont = font(20))
#     frame(anim)
# end

# gif(anim, "$(OUTPUT_NAME)/energy-evolution.gif", fps=15)

# anim = Animation()
# for i = 1:length(t)
#     p = plot(t, mv, lw=plotw)
#     plot!(p, t[i:end], mv[i:end],linecolor=:white,lw=hidew)
#     frame(anim)
# end

# gif(anim, "$(OUTPUT_NAME)/momentum-evolution.gif", fps=15)


# plot(t, mv,lw=plotw,xtickfont = font(20),ytickfont = font(20), size=(800,600),
#         xlabel="Time (s)",ylabel="Magnitude of momentum (kg m/s)",xguidefontsize=20,yguidefontsize=20,xlim=[0,2],ylim=[0,10])
# savefig("$(OUTPUT_NAME)/momentum-evolution.png")

# t = []
# mv = []
# open("$(OUTPUT_NAME)/statistics_momentum.dat") do f
#     line = 0

#     while !eof(f)
#         line += 2
#         append!(t, parse(Float64, readline(f)))
#         append!(mv, parse(Float64, readline(f)))
#     end
# end

# # pingpong
# plot(t[1:3040], mv[1:3040],lw=plotw,xtickfont = font(20),ytickfont = font(20),size=(800,600),
#         xlabel="Time (s)",ylabel="Magnitude of momentum (kg m/s)",xguidefontsize=20,yguidefontsize=20,xlim=[0,1.5],ylim=[0,100])
# savefig("$(OUTPUT_NAME)/momentum-evolution.png")



# anim = Animation()
# for i = 1:length(t)
#     p = plot(t, ma, lw=plotw)
#     plot!(p, t[i:end], ma[i:end],linecolor=:white,lw=hidew)
#     frame(anim)
# end

# gif(anim, "$(OUTPUT_NAME)/angular-momentum-evolution.gif", fps=15)


# anim = Animation()
# Etmp = Ek+Es
# for i = 1:length(t)
#     p = plot(t, Etmp, lw=plotw)
#     plot!(p, t[i:end], Etmp[i:end],linecolor=:white,lw=hidew)
#     frame(anim)
# end

# gif(anim, "$(OUTPUT_NAME)/kinetic-and-strain-energy-evolution.gif", fps=15)

# anim = Animation()
# for i = 1:length(t)
#     # p = plot(t, 0.1*pn, lw=plotw)
#     # plot!(p, t[i:end], 0.1*pn[i:end],linecolor=:white,lw=hidew)
#     p = plot(t, pt, lw=plotw)
#     plot!(p, t[i:end], pt[i:end],linecolor=:white,lw=hidew)
#     # plot!(t, gt, lw=plotw)
#     # plot!(p, t[i:end], gt[i:end],linecolor=:white,lw=hidew)
#     # p = plot(t, pt, lw=plotw)
#     # plot!(p, t[i:end], pt[i:end],linecolor=:white,lw=hidew)
#     # plot!(t, gt, lw=plotw)
#     # plot!(p, t[i:end], gt[i:end],linecolor=:white,lw=hidew)
#     frame(anim)
# end

# gif(anim, "$(OUTPUT_NAME)/impulse-bc.gif", fps=15)

# anim = Animation()
# for i = 1:length(t)
#     # p = plot(t, 0.1*pn, lw=plotw)
#     # plot!(p, t[i:end], 0.1*pn[i:end],linecolor=:white,lw=hidew)
#     p = plot(t, pt, lw=plotw)
#     plot!(p, t[i:end], pt[i:end],linecolor=:white,lw=hidew)
#     # plot!(t, gt, lw=plotw)
#     # plot!(p, t[i:end], gt[i:end],linecolor=:white,lw=hidew)
#     # p = plot(t, pt, lw=plotw)
#     # plot!(p, t[i:end], pt[i:end],linecolor=:white,lw=hidew)
#     # plot!(t, gt, lw=plotw)
#     # plot!(p, t[i:end], gt[i:end],linecolor=:white,lw=hidew)
#     frame(anim)
# end

# gif(anim, "$(OUTPUT_NAME)/impulse-bc.gif", fps=15)

# # Slope
# p = plot(t, pt, lw=plotw,xtickfont = font(20),ytickfont = font(20),size=(900,600),
#          xlabel="Time (s)",ylabel="Magnitude of Impulse (N s)",xguidefontsize=20,yguidefontsize=20,xlim=[0,1],ylim=[0,2.5e-5],labels="Tangential impulse: wall",label=true,
#          legendfontsize=20,legend=:right)
# plot!(t, gt, lw=plotw,labels="Tangential impulse: gravity")

# savefig("$(OUTPUT_NAME)/impulse-bc.png");


# time = []
# impulsen = []
# impulset = []
# extimpulsen = []
# extimpulset = []
# open("$(OUTPUT_NAME)/statistics_impulse.dat") do f
#     line = 0

#     while !eof(f)
#         line += 5
#         append!(time, parse(Float64, readline(f)))
#         append!(impulsen, parse(Float64, readline(f)))
#         append!(impulset, parse(Float64, readline(f)))
#         append!(extimpulsen, parse(Float64, readline(f)))
#         append!(extimpulset, parse(Float64, readline(f)))
#     end
# end


# anim = Animation()
# for i = 1:length(time)
#     # p = plot(t, pn, lw=plotw)
#     # plot!(p, t[i:end], pn[i:end],linecolor=:white,lw=hidew)
#     # plot!(t, pt, lw=plotw)
#     # plot!(p, t[i:end], pt[i:end],linecolor=:white,lw=hidew)
#     # plot!(t, gt, lw=plotw)
#     # plot!(p, t[i:end], gt[i:end],linecolor=:white,lw=hidew)
#     p = plot(time, impulset, lw=plotw)
#     plot!(p, time[i:end], impulset[i:end],linecolor=:white,lw=hidew)
#     plot!(time, extimpulset, lw=plotw)
#     plot!(p, time[i:end], extimpulset[i:end],linecolor=:white,lw=hidew)
#     frame(anim)
# end

# gif(anim, "$(OUTPUT_NAME)/impulse-bc-fine.gif", fps=10000)


OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/shaking-sphere/outputs-mu.1"
# gripper
time = []
impulsen = []
impulset = []
extimpulsen = []
extimpulset = []
open("$(OUTPUT_NAME)/statistics_impulse.dat") do f
    line = 0

    while !eof(f)
        line += 5
        append!(time, parse(Float64, readline(f)))
        append!(impulsen, parse(Float64, readline(f)))
        append!(impulset, parse(Float64, readline(f)))
        append!(extimpulsen, parse(Float64, readline(f)))
        append!(extimpulset, parse(Float64, readline(f)))
    end
end

t = time[1:200:end]
it = impulset[1:200:end]
p = plot(t, it, lw=5,xtickfont = font(20),ytickfont = font(20),size=(900,600),
         xlabel="Time (s)",ylabel="Impulse (N s)",xguidefontsize=20,yguidefontsize=20,xlim=[0,4],ylim=[-0.02,0.02],labels="Impulse from grippers in vertical direction",label=true,
         legendfontsize=20,legend=:topright)

savefig("$(OUTPUT_NAME)/impulse-bc.png");


# time = time[27100:27600]
# impulset = impulset[27100:27600]
# p = plot(time, impulset, lw=5,xtickfont = font(20),ytickfont = font(20),size=(900,600),
#          xlabel="Time (s)",ylabel="Impulse (N s)",xguidefontsize=20,yguidefontsize=20,xlim=[time[1],time[end]],ylim=[-0.01,0.01],labels="Impulse in vertical direction",label=true,
#          legendfontsize=20,legend=:topright)

# savefig("$(OUTPUT_NAME)/impulse-bc-zoom.png");

#=
E8 = []
E4 = []
E2 = []

t = []
OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/rotating-sphere/outputs-CFL.2"
open("$(OUTPUT_NAME)/statistics.dat") do f
    line = 0

    while !eof(f)
        line += 13
        append!(t, parse(Float64, readline(f)))
        append!(s, parse(Float64, readline(f)))
        append!(m, parse(Float64, readline(f)))
        append!(Ek, parse(Float64, readline(f)))
        append!(Es, parse(Float64, readline(f)))
        append!(Ep, parse(Float64, readline(f)))
        append!(E2, parse(Float64, readline(f)))
        append!(mv, parse(Float64, readline(f)))
        append!(ma, parse(Float64, readline(f)))
        append!(pn, parse(Float64, readline(f)))
        append!(pt, parse(Float64, readline(f)))
        append!(gn, parse(Float64, readline(f)))
        append!(gt, parse(Float64, readline(f)))
    end
end

t = []
OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/rotating-sphere/outputs-CFL.4"
open("$(OUTPUT_NAME)/statistics.dat") do f
    line = 0

    while !eof(f)
        line += 13
        append!(t, parse(Float64, readline(f)))
        append!(s, parse(Float64, readline(f)))
        append!(m, parse(Float64, readline(f)))
        append!(Ek, parse(Float64, readline(f)))
        append!(Es, parse(Float64, readline(f)))
        append!(Ep, parse(Float64, readline(f)))
        append!(E4, parse(Float64, readline(f)))
        append!(mv, parse(Float64, readline(f)))
        append!(ma, parse(Float64, readline(f)))
        append!(pn, parse(Float64, readline(f)))
        append!(pt, parse(Float64, readline(f)))
        append!(gn, parse(Float64, readline(f)))
        append!(gt, parse(Float64, readline(f)))
    end
end

t = []
OUTPUT_NAME = "/home/yiminlin/Desktop/drake/multibody/fem/mpm-dev/examples/rotating-sphere/outputs-CFL.8"
open("$(OUTPUT_NAME)/statistics.dat") do f
    line = 0

    while !eof(f)
        line += 13
        append!(t, parse(Float64, readline(f)))
        append!(s, parse(Float64, readline(f)))
        append!(m, parse(Float64, readline(f)))
        append!(Ek, parse(Float64, readline(f)))
        append!(Es, parse(Float64, readline(f)))
        append!(Ep, parse(Float64, readline(f)))
        append!(E8, parse(Float64, readline(f)))
        append!(mv, parse(Float64, readline(f)))
        append!(ma, parse(Float64, readline(f)))
        append!(pn, parse(Float64, readline(f)))
        append!(pt, parse(Float64, readline(f)))
        append!(gn, parse(Float64, readline(f)))
        append!(gt, parse(Float64, readline(f)))
    end
end

p = plot(t, E8, lw=5,xtickfont = font(20),ytickfont = font(20),size=(900,600),
         xlabel="Time (s)",ylabel="Kinetic energy ( J )",xguidefontsize=20,yguidefontsize=20,xlim=[0,2],labels="Time step size 4E-4 s",label=true,
         legendfontsize=20,legend=:topright)
plot!(t, E4, lw=5,xtickfont = font(20),ytickfont = font(20),size=(900,600),
         xlabel="Time (s)",ylabel="Kinetic energy ( J )",xguidefontsize=20,yguidefontsize=20,xlim=[0,2],labels="Time step size 2E-4 s",label=true,
         legendfontsize=20,legend=:topright)
plot!(t, E2, lw=5,xtickfont = font(20),ytickfont = font(20),size=(900,600),
         xlabel="Time (s)",ylabel="Kinetic energy ( J )",xguidefontsize=20,yguidefontsize=20,xlim=[0,2],labels="Time step size 1E-4 s",label=true,
         legendfontsize=20,legend=:topright)

savefig("/home/yiminlin/Desktop/CFL-plot.png");

=#