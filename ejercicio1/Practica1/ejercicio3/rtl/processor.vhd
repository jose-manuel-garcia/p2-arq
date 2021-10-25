--------------------------------------------------------------------------------
-- Procesador MIPS con pipeline curso Arquitectura 2021-2022
--
-- (INCLUIR AQUI LA INFORMACION SOBRE LOS AUTORES)
-- Jose Manuel Garcia Giraldez
-- Ruben Fernandez Aliman
--
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

entity processor is
   port(
      Clk         : in  std_logic; -- Reloj activo en flanco subida
      Reset       : in  std_logic; -- Reset asincrono activo nivel alto
      -- Instruction memory
      IAddr      : out std_logic_vector(31 downto 0); -- Direccion Instr
      IDataIn    : in  std_logic_vector(31 downto 0); -- Instruccion leida
      -- Data memory
      DAddr      : out std_logic_vector(31 downto 0); -- Direccion
      DRdEn      : out std_logic;                     -- Habilitacion lectura
      DWrEn      : out std_logic;                     -- Habilitacion escritura
      DDataOut   : out std_logic_vector(31 downto 0); -- Dato escrito
      DDataIn    : in  std_logic_vector(31 downto 0)  -- Dato leido
      
   );
end processor;

architecture rtl of processor is

  component alu
    port(
      OpA      : in std_logic_vector (31 downto 0);
      OpB      : in std_logic_vector (31 downto 0);
      Control  : in std_logic_vector (3 downto 0);
      Result   : out std_logic_vector (31 downto 0);
      Signflag : out std_logic;
      Zflag    : out std_logic
    );
  end component;

  component reg_bank
     port (
        Clk   : in std_logic; -- Reloj activo en flanco de subida
        Reset : in std_logic; -- Reset as�ncrono a nivel alto
        A1    : in std_logic_vector(4 downto 0);   -- Direcci�n para el puerto Rd1
        Rd1   : out std_logic_vector(31 downto 0); -- Dato del puerto Rd1
        A2    : in std_logic_vector(4 downto 0);   -- Direcci�n para el puerto Rd2
        Rd2   : out std_logic_vector(31 downto 0); -- Dato del puerto Rd2
        A3    : in std_logic_vector(4 downto 0);   -- Direcci�n para el puerto Wd3
        Wd3   : in std_logic_vector(31 downto 0);  -- Dato de entrada Wd3
        We3   : in std_logic -- Habilitaci�n de la escritura de Wd3
     );
  end component reg_bank;

  component control_unit
     port (
        -- Entrada = codigo de operacion en la instruccion:
        OpCode   : in  std_logic_vector (5 downto 0);
        jump : out std_logic;
        -- Seniales para el PC
        Branch   : out  std_logic; -- 1 = Ejecutandose instruccion branch
        -- Seniales relativas a la memoria
        MemToReg : out  std_logic; -- 1 = Escribir en registro la salida de la mem.
        MemWrite : out  std_logic; -- Escribir la memoria
        MemRead  : out  std_logic; -- Leer la memoria
        -- Seniales para la ALU
        ALUSrc   : out  std_logic;                     -- 0 = oper.B es registro, 1 = es valor inm.
        ALUOp    : out  std_logic_vector (2 downto 0); -- Tipo operacion para control de la ALU
        -- Seniales para el GPR
        RegWrite : out  std_logic; -- 1 = Escribir registro
        RegDst   : out  std_logic  -- 0 = Reg. destino es rt, 1=rd
     );
  end component;

  component alu_control is
   port (
      -- Entradas:
      ALUOp  : in std_logic_vector (2 downto 0); -- Codigo de control desde la unidad de control
      Funct  : in std_logic_vector (5 downto 0); -- Campo "funct" de la instruccion
      -- Salida de control para la ALU:
      ALUControl : out std_logic_vector (3 downto 0) -- Define operacion a ejecutar por la ALU
   );
 end component alu_control;


  --////////////////////////////////////////////////////--
  --////////////////////////////////////////////////////--
  --//////////////DECLARACION DE VARIABLES//////////////--
  --////////////////////////////////////////////////////--

  -------------------------------------------------------
  --------------------etapa IF---------------------------

  signal PC_next        : std_logic_vector(31 downto 0);
  signal PC_reg         : std_logic_vector(31 downto 0);
  signal PC_plus4       : std_logic_vector(31 downto 0);
  signal Instruction    : std_logic_vector(31 downto 0); -- La instrucción desde lamem de instr

  -------------------------------------------------------
  -------------------------------------------------------


  -------------------------------------------------------
  --------------------IF/ID------------------------------
 
  signal PC_plus4_ID    : std_logic_vector(31 downto 0);
  signal Instruccion_ID : std_logic_vector(31 downto 0); --instruccion 
  signal enable_IF_ID   : std_logic;

  -------------------------------------------------------
  -------------------------------------------------------



  -------------------------------------------------------
  --------------------etapa ID---------------------------

  signal Ctrl_Jump_ID, Ctrl_Branch_ID, Ctrl_MemWrite_ID, Ctrl_MemRead_ID,  Ctrl_ALUSrc_ID, Ctrl_RegDest_ID, Ctrl_MemToReg_ID, Ctrl_RegWrite_ID : std_logic; --señales de control
  signal reg_RS_ID, reg_RT_ID : std_logic_vector(31 downto 0); 
  signal reg_RD_ID       : std_logic_vector(4 downto 0);
  signal Inm_ext_ID        : std_logic_vector(31 downto 0); -- La parte baja de la instrucción extendida de signo
  signal Ctrl_ALUOP_ID     : std_logic_vector(2 downto 0);
  

  -------------------------------------------------------
  -------------------------------------------------------


  ------------------------ID/EX--------------------------
  -------------------------------------------------------

  signal PC_plus4_EX    : std_logic_vector(31 downto 0);
  signal Instruccion_EX : std_logic_vector(31 downto 0); --instruccion
  signal enable_ID_EX   : std_logic; --señal que activa el registro ID/EX
  -------------------------------------------------------
  -------------------------------------------------------

  -------------------------EX----------------------------
  -------------------------------------------------------

  signal Ctrl_Jump_EX, Ctrl_Branch_EX, Ctrl_MemWrite_EX, Ctrl_MemRead_EX,  Ctrl_ALUSrc_EX, Ctrl_MemToReg_EX, Ctrl_RegWrite_EX, Ctrl_RegDest_EX : std_logic; --señales de control 
  signal reg_RS_EX, reg_RT_EX : std_logic_vector(31 downto 0);
  signal reg_RD_EX       : std_logic_vector(4 downto 0);
  signal Alu_Op2_EX      : std_logic_vector(31 downto 0); --segundo operando de la alu
  signal Alu_Op1_EX      : std_logic_vector(31 downto 0); --primer operando de la alu
  signal AluControl   : std_logic_vector(3 downto 0); 
  signal Ctrl_ALUOP_EX     : std_logic_vector(2 downto 0); -- ALU op
  signal Inm_ext_EX       : std_logic_vector(31 downto 0); -- La parte baja de la instrucción extendida de signo
  signal Alu_Result_EX        : std_logic_vector(31 downto 0); --resultado de la alu
  signal ALU_Igual    : std_logic; --Bandera que se activa cuando el resultado de la alu es 0
  signal Addr_Jump      : std_logic_vector(31 downto 0); --direccion de salto en caso de salto incondicional
  signal Addr_Branch_EX    : std_logic_vector(31 downto 0); --direccion de salto en caso de salto condicional

  -------------------------------------------------------
  -------------------------------------------------------

  -----------------------EX/EM---------------------------
  -------------------------------------------------------

  signal enable_EX_MEM   : std_logic; --señal que activa el registro EX/MEM
  
  -------------------------------------------------------
  -------------------------------------------------------

  -------------------------MEM---------------------------
  -------------------------------------------------------

  signal Addr_Jump_MEM      : std_logic_vector(31 downto 0); --direccion de salto incondicional (se calcula en la etapa EX)
  signal branch_MEM      : std_logic; -- PCSRC, resultado de la AND que confirma o no el salto condicional
  signal Addr_Branch_MEM    : std_logic_vector(31 downto 0); --direccion de salto condicional (se calcula en la etapa EX)
  signal Ctrl_Branch_MEM, Ctrl_MemWrite_MEM, Ctrl_MemRead_MEM, Ctrl_MemToReg_MEM, Ctrl_RegWrite_MEM, Ctrl_Jump_MEM : std_logic; -- señales de control necesarias para la etapa MEM
  signal reg_RD_MEM      : std_logic_vector(4 downto 0);
  signal reg_RT_MEM      : std_logic_vector(31 downto 0);
  signal Alu_Result_MEM        : std_logic_vector(31 downto 0); --resultado de la alu
  signal ALU_Igual_MEM    : std_logic; --Bandera que se activa cuando el resultado de la alu es 0
  signal dataIn_MEM     : std_logic_vector(31 downto 0); --From Data Memory



  -------------------------------------------------------
  -------------------------------------------------------

  -----------------------MEM/WB--------------------------
  -------------------------------------------------------

  signal enable_MEM_WB   : std_logic; --señal que activa el registro MEM/WB
  

  -------------------------------------------------------
  -------------------------------------------------------


  -----------------------WB------------------------------
  -------------------------------------------------------

  signal Ctrl_MemToReg_WB, Ctrl_RegWrite_WB : std_logic;  -- señales de control necesarias para la etapa WB
  signal Alu_Result_WB        : std_logic_vector(31 downto 0); --resultado de la alu
  signal reg_RD_WB      : std_logic_vector(4 downto 0);
  signal reg_RD_data  : std_logic_vector(31 downto 0);
  signal dataIn_WB     : std_logic_vector(31 downto 0); --From Data Memory

  -------------------------------------------------------
  -------------------------------------------------------

  ----------------------Forwarding-----------------------
  -------------------------------------------------------

  signal forward_opA			  : std_logic_vector(1 downto 0); --mux del operando A de la alu
  signal forward_opB			  : std_logic_vector(1 downto 0); --mux del operando B de la alu

  signal reg_RS_ID_AUX : std_logic_vector(31 downto 0); --auxiliar para conservar el valor de RS antes forwading
  signal reg_RT_ID_AUX   : std_logic_vector(31 downto 0);--auxiliar para conservar el valor de RT antes de forwading
								     
  -------------------------------------------------------
  -------------------------------------------------------


  ---------------------------Hazard----------------------
  -------------------------------------------------------

  signal hazard          : std_logic;
  signal actualizar_PC          : std_logic;
  -------------------------------------------------------
  -------------------------------------------------------
  

 
  

begin


  ---------------------------------------------------------
  ----------------------------IF---------------------------


  -- calculo de la proxima direccion de PC  
  PC_next <=  Addr_Jump_MEM when Ctrl_Jump_MEM = '1' else 
              Addr_Branch_MEM when Branch_MEM = '1' else PC_plus4;

  PC_reg_proc: process(Clk, Reset)

  begin

    if Reset = '1' then
      PC_reg <= (others => '0');

    elsif rising_edge(Clk) then
	if actualizar_PC = '1' then
      	   PC_reg <= PC_next;

    end if;

  end process;


  PC_plus4    <= PC_reg + 4;
  IAddr       <= PC_reg;
  Instruction <= IDataIn;

   --evita que se actualice el reloj en caso de hazard
   actualizar_PC <= '1' when hazard = '0' else '0';

  ----------------------------------------------------------
  ----------------------------------------------------------


  ----------------------------IF/ID-------------------------
  ----------------------------------------------------------
  IF_ID_Regs: process(clk,reset)

  begin

    if reset = '1' then
     PC_plus4_ID <= (others=>'0');
     instruccion_ID <= (others => '0');

    elsif rising_edge(clk) and enable_IF_ID= '1' and hazard_control = '0' then
     PC_plus4_ID <= PC_plus4;
     instruccion_ID <= instruction;

    end if;

  end process;

  enable_IF_ID <= '1'; --mantenemos el valor siempre a uno hasta posteriores practicas


  -----------------------------------------------------------
  -----------------------------------------------------------

  -----------------------------------------------------------
  -----------------------------ID----------------------------

  --comprobacion de si se activa hazard
  -- el metodo a utilizar no es optimo pues genera burbujas innecesarias, funciona comprobando que el registro rs o rt no coincidan con el destino rt de la nueva funcion
  hazard <= '1' when Ctrl_MemRead_EX = '1' and (Instruccion_EX(20 downto 16) = Instruccion_ID(25 downto 21) or Instruccion_EX(20 downto 16) = Instruccion_ID(20 downto 16)) else
                    '0';

  RegsMIPS : reg_bank
  port map (
    Clk   => Clk,
    Reset => Reset,
    A1    => Instruccion_ID(25 downto 21),
    Rd1   => reg_RS_ID,
    A2    => Instruccion_ID(20 downto 16),
    Rd2   => reg_RT_ID,
    A3    => reg_RD_WB, --<------------------------------------------------
    Wd3   => reg_RD_data,
    We3   => Ctrl_RegWrite_WB
  );

  UnidadControl : control_unit
  port map(
    OpCode   => Instruccion_ID(31 downto 26),
    -- Señales para el PC
    Jump   => Ctrl_Jump_ID,
    Branch   => Ctrl_Branch_ID,
    -- Señales para la memoria
    MemToReg => Ctrl_MemToReg_ID,
    MemWrite => Ctrl_MemWrite_ID,
    MemRead  => Ctrl_MemRead_ID,
    -- Señales para la ALU
    ALUSrc   => Ctrl_ALUSrc_ID,
    ALUOP    => Ctrl_ALUOP_ID,
    -- Señales para el GPR
    RegWrite => Ctrl_RegWrite_ID,
    RegDst   => Ctrl_RegDest_ID
  );

  -- calculo del valor inmediato extendido en signo
  Inm_ext_ID        <= x"FFFF" & Instruccion_ID(15 downto 0) when Instruccion_ID(15)='1' else
                    x"0000" & Instruccion_ID(15 downto 0);  

  
  --Hazard detention unit

  hazard_control <= '1' when EX_Ctrl_MemRead = '1' and (EX_reg_RT_number = ID_reg_RS_number or EX_reg_RT_number = ID_reg_RT_number) else
                    '0';


  -----------------------------------------------------------------
  -----------------------------------------------------------------


  --//////////////////////////////////////////////////////////////--
  --//////////////////////////////////////////////////////////////--
  --//////////////////////////////////////////////////////////////--


  ----------------------------ID/EX--------------------------------
  -----------------------------------------------------------------

 

  ID_EX_Regs: process(clk,reset)

  begin

    if reset = '1' then
     Instruccion_EX <= (others=>'0');
     PC_plus4_EX <= (others=>'0');
     instruccion_EX <= (others=>'0');
     Ctrl_Jump_EX <= '0';
     Ctrl_Branch_EX <= '0';
     Ctrl_MemWrite_EX <= '0';
     Ctrl_MemRead_EX <= '0';
     Ctrl_ALUSrc_EX <= '0';
     Ctrl_MemToReg_EX <= '0';
     Ctrl_RegWrite_EX <= '0';
     Ctrl_RegDest_EX <= '0';
     reg_RS_EX <= (others=>'0');
     reg_RT_EX <= (others=>'0');
     Ctrl_ALUOP_EX <= (others=>'0');
     Inm_ext_EX <= (others=>'0');
     reg_RT_ID_AUX <= (others=>'0');
     reg_RS_ID_AUX <= (others=>'0');
    


    elsif rising_edge(clk) and enable_ID_EX='1' then
	if hazard = '1' then       --si el hazard esta activado necesitamos generar instrucciones nop, para ello
	  Ctrl_Jump_EX <= '0';	   --todas las entradas de control a 0
     	  Ctrl_Branch_EX <= '0';
     	  Ctrl_MemWrite_EX <= '0';
     	  Ctrl_MemRead_EX <= '0';
     	  Ctrl_ALUSrc_EX <= '0';
     	  Ctrl_MemToReg_EX <= '0';
     	  Ctrl_RegWrite_EX <= '0';
     	  Ctrl_RegDest_EX <= '0';
	  Ctrl_ALUOP_EX <= (others=>'0');
	elsif hazard = '0' then
     	  Instruccion_EX <= Instruccion_ID;
     	  PC_plus4_EX <= PC_plus4_ID;
     	  instruccion_EX <= instruccion_ID;
     	  Ctrl_Jump_EX <= Ctrl_Jump_ID ;
     	  Ctrl_Branch_EX <= Ctrl_Branch_ID;
     	  Ctrl_MemWrite_EX <= Ctrl_MemWrite_ID;
    	  Ctrl_MemRead_EX <= Ctrl_MemRead_ID;
     	  Ctrl_ALUSrc_EX <= Ctrl_ALUSrc_ID;
     	  Ctrl_MemToReg_EX <= Ctrl_MemToReg_ID;
  	  Ctrl_RegWrite_EX <= Ctrl_RegWrite_ID;
   	  reg_RS_EX <= reg_RS_ID;
          reg_RT_EX <= reg_RT_ID;
    	  Ctrl_ALUOP_EX <= Ctrl_ALUOP_ID;
    	  Inm_ext_EX <= Inm_ext_ID;
    	  Ctrl_RegDest_EX <= Ctrl_RegDest_ID;
     	  reg_RT_ID_AUX <= reg_RT_ID
     	  reg_RS_ID_AUX <= reg_RS_ID

    
	



    end if;

  end process;

  enable_ID_EX <= '1'; --mantenemos el valor siempre a uno hasta posteriores practicas

  -----------------------------------------------------------------
  -----------------------------------------------------------------


  --//////////////////////////////////////////////////////////////--
  --//////////////////////////////////////////////////////////////--
  --//////////////////////////////////////////////////////////////--

 
  -------------------------------EX--------------------------------
  -----------------------------------------------------------------

  -- calculo del registro de destino
  reg_RD_EX     <= Instruccion_EX(20 downto 16) when Ctrl_RegDest_EX = '0' else Instruccion_EX(15 downto 11);


----------------------------------------en esta etapa se comprobara forwading--------------------------------------

  -- 01 si el registro destino en la etapa mem coincide con el operando A de la etapa EX
  -- 10 si el registro destino destino de la etapa wb coincide con el operando A de la etapa EX
  -- 00 si el operando de la etapa A es RS
  forward_opA <= "01" when Ctrl_RegWrite_MEM = '1' and reg_RD_MEM = Instruccion_EX(25 downto 21)  and Instruccion_EX(25 downto 21) /= x"0000" else "10" when Ctrl_RegWrite_WB = '1' and reg_RD_WB = Instruccion_EX(25 downto 21)  and Instruccion_EX(25 downto 21) /= x"0000" else
			 "00";

  -- calculo del primer operando de la alu
  Alu_Op1_EX <= reg_RS_EX when forward_opA = "00" else Alu_Result_MEM when forward_opA = "01" else reg_RD_data;
  reg_RS_EX <= Alu_Op1_EX
  

  -- 01 si el registro destino en la etapa mem coincide con el registro RT de la etapa EX
  -- 10 si el registro destino destino de la etapa wb coincide con el registro RT de la etapa EX
  -- 00 si el operando de la etapa B es RT o inmediato
  forward_opB <= "01" when Ctrl_RegWrite_MEM = '1' and reg_RD_MEM = Instruccion_EX(20 downto 16)  and Instruccion_EX(20 downto 16) /= x"0000" else "10" when Ctrl_RegWrite_WB = '1' and reg_RD_WB = Instruccion_EX(20 downto 16)  and Instruccion_EX(20 downto 16) /= x"0000" else
			 "00";
  reg_RT_EX <= reg_RT_ID_AUX when forward_opB = "00" else Alu_Result_MEM when forward_opB = "01" else reg_RD_data;

  -- calculo del segundo operando de la alu
  Alu_Op2_EX <= reg_RT_EX when Ctrl_ALUSrc_EX = '0' else Inm_ext_EX;

--------------------------------------------------------------------------------------------------------------------


--------------adelantamiento para el caso de 3 ciclos, en el banco de registros(etapa ID)---------------

  reg_RS_ID <= reg_RD_data when Ctrl_RegWrite_WB = '1' and reg_RD_WB = Instruccion_ID(25 downto 21) and Instruccion_ID(25 downto 21) else reg_RS_ID_AUX;
			   
  reg_RT_ID <= reg_RD_data when Ctrl_RegWrite_WB = '1' and reg_RD_WB = Instruccion_ID(20 downto 16) and Instruccion_ID(20 downto 16) else reg_RS_ID_AUX;

-------------------------------------------------------------------------------------------------------

  --instanciacion

  Alu_control_i: alu_control
  port map(
    -- Entradas:
    ALUOp  => Ctrl_ALUOP_EX, -- Codigo de control desde la unidad de control
    Funct  => instruccion_EX (5 downto 0), -- Campo "funct" de la instruccion
    -- Salida de control para la ALU:
    ALUControl => AluControl -- Define operacion a ejecutar por la ALU
  );

  Alu_MIPS : alu
  port map (
    OpA      => Alu_Op1_EX,
    OpB      => Alu_Op2_EX,
    Control  => AluControl,
    Result   => Alu_Result_EX,
    Signflag => open,
    Zflag    => ALU_IGUAL
  );
 

  Addr_Jump      <= PC_plus4_EX(31 downto 28) & Instruccion_EX(25 downto 0) & "00"; --calculo direccion de salto incondicional
  
  Addr_Branch_EX   <= PC_plus4_EX + ( Inm_ext_EX(29 downto 0) & "00");--calculo direccion de salto BEQ

 
 
  
  -----------------------------------------------------------------
  -----------------------------------------------------------------

  -----------------------------EX/MEM-------------------------------
  -----------------------------------------------------------------

  
 ---pasarle jump

  EX_MEM_Regs: process(clk,reset)

  begin

    if reset = '1' then
     Ctrl_Branch_MEM <= '0';
     Ctrl_MemWrite_MEM <= '0';
     Ctrl_MemRead_MEM <= '0';
     Ctrl_MemToReg_MEM <= '0';
     Ctrl_RegWrite_MEM <= '0';
     Ctrl_Jump_MEM <= '0';
     Alu_Result_MEM <= (others=>'0');
     Addr_Branch_MEM <= (others => '0');
     ALU_Igual_MEM <= '0';
     reg_RD_MEM <= (others => '0');
     Addr_Jump_MEM <= (others => '0');
     reg_RT_MEM <= (others => '0');
     
     

    elsif rising_edge(clk) and enable_EX_MEM='1' then
     Ctrl_Branch_MEM <= Ctrl_Branch_EX;
     Alu_Result_MEM <= Alu_Result_EX;
     Addr_Branch_MEM <= Addr_Branch_EX;
     ALU_Igual_MEM <= ALU_Igual;
     reg_RD_MEM <= reg_RD_EX;
     Ctrl_MemWrite_MEM <= Ctrl_MemWrite_EX;
     Ctrl_MemRead_MEM <= Ctrl_MemRead_EX;
     Ctrl_MemToReg_MEM <= Ctrl_MemToReg_EX;
     Ctrl_RegWrite_MEM <= Ctrl_RegWrite_EX;
     Ctrl_Jump_MEM <= Ctrl_Jump_EX;
     Addr_Jump_MEM <= Addr_Jump;
     reg_RT_MEM <= reg_RT_EX;


    end if;

  end process;

  enable_EX_MEM <= '1'; --mantenemos el valor siempre a uno hasta posteriores practicas


  -----------------------------------------------------------------
  -----------------------------------------------------------------

  -------------------------------MEM-------------------------------
  -----------------------------------------------------------------


  branch_MEM <= ALU_Igual_MEM and Ctrl_Branch_MEM; --PCSRC
  DAddr      <= Alu_Result_MEM; 
  DDataOut   <= reg_RT_MEM;
  DWrEn      <= Ctrl_MemWrite_MEM;
  dRdEn      <= Ctrl_MemRead_MEM;
  dataIn_MEM <= DDataIn;


  -----------------------------------------------------------------
  -----------------------------------------------------------------

  -------------------------------MEM/WB----------------------------
  -----------------------------------------------------------------

  MEM_WB_Regs: process(clk,reset)

  begin

    if reset = '1' then
     Ctrl_MemToReg_WB <= '0';
     Ctrl_RegWrite_WB <= '0';
     Alu_Result_WB <= (others=>'0');
     reg_RD_WB <= (others=>'0');
     dataIn_WB <= (others => '0');
     

    elsif rising_edge(clk) and enable_MEM_WB='1' then
     Ctrl_MemToReg_WB <= Ctrl_MemToReg_MEM;
     Ctrl_RegWrite_WB <= Ctrl_RegWrite_MEM;
     Alu_Result_WB <= Alu_Result_MEM;
     reg_RD_WB <= reg_RD_MEM;
     dataIn_WB <= dataIn_MEM;
 


    end if;

  end process;

  enable_MEM_WB <= '1'; --mantenemos el valor siempre a uno hasta posteriores practicas


  -----------------------------------------------------------------
  -----------------------------------------------------------------
 
  ----------------------------WB-----------------------------------
  -----------------------------------------------------------------

  reg_RD_data <= dataIn_WB when Ctrl_MemToReg_WB = '1' else Alu_Result_WB; 

  -----------------------------------------------------------------
  -----------------------------------------------------------------

end architecture;
