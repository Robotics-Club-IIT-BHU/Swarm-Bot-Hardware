# DotBotHardware
copy paste the following into .bashrc
```bash
$ gedit ~/.bashrc
```
```
export PI1=192.168.43.4
export PI2=192.168.43.54
export PI3=192.168.43.118
export PI4=192.168.43.142

export NPI1=192.168.1.2
export NPI2=192.168.1.1
export NPI3=192.168.1.1
export NPI4=192.168.1.1

alias pi1-connect='ssh pi@$PI1'
alias pi2-connect='ssh pi@$PI2'
alias pi3-connect='ssh pi@$PI3'
alias pi4-connect='ssh pi@$PI4'

alias npi1-connect='ssh pi@$NPI1'
alias npi2-connect='ssh pi@$NPI2'
alias npi3-connect='ssh pi@$NPI3'
alias npi4-connect='ssh pi@$NPI4'
```
