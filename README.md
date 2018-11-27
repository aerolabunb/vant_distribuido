# vant_distribuido

Este repositório agrega os códigos desenvolvidos em C para o controle distribudo de múltiplos veículos aéreos não tripulados (VANT). Este controlador será executado em uma plataforma Gumstix e faz conexões com um Modem de longo alcance e um piloto
automático Pixhawk.

O repositório principal é o 'main' que contém os códigos em C. As funções princpais são definas no arquivo 'main.c', funções de 
inicialização estão em 'system_init.c' e comandos de voo em 'commands.c'. As comunicações são realizados via protocolo Mavlink (versão 1)
cuja biblioteca pode ser encontrada em 'mavlink/common'. Os dados do datalogger são incluídos em 'matlabdatafiles' e o código para adição de dados em formato '.mat' ficao no diretório 'gdatalogger'.

Para compilar o sistema basta, dentro da main, utilizar o comando 'make all' e executar o arquivo 'main'.
