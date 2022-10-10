
.data					# section declaration

        # Useful offset constants for accessing members of a 
        # struct mp1_blink_struct structure
        LOCATION   = 0    
        ON_CHAR    = 2
        OFF_CHAR   = 3 
        ON_LENGTH  = 4
        OFF_LENGTH = 6
        COUNTDOWN  = 8
        STATUS     = 10
        NEXT       = 12

        STRUCT_SIZE = 16

# Pointer to head of list (initialized to NULL)
mp1_list_head:
        .long   0

.text					# section declaration

# Export the function symbol names

.global mp1_rtc_tasklet
.global mp1_ioctl

# ##############################################################################

# void mp1_poke(void);
#
# Interface: Register-based arguments (not C-style)
#    Inputs: %cl  - The byte you wish to write
#            %eax - Offset from the start of video memory that you wish
#                   to write to
#   Outputs: Text-mode video screen is written to at location %eax with
#            the byte in %cl
# Registers: Clobbers EDX
mp1_poke:
        
	movl    vmem_base_addr(,1),%edx
	movb    %cl,(%edx,%eax,1)
	ret

# ##############################################################################

# void mp1_rtc_tasklet(unsigned long);
#
# Interface: Update the state of the game
#    Inputs: unsigned long
#    Outputs: None
#    Register: Clobbers EAX
mp1_rtc_tasklet:
	pushl	%ebp			
	movl	%esp, %ebp		
	pushl	%ebx			
	pushl	%esi
	pushl	%edi
        movl    mp1_list_head, %ebx
        cmpl    $0, %ebx
        je      over

Main:
        decw    COUNTDOWN(%ebx)
        cmpw    $0, COUNTDOWN(%ebx)
        jne     Next_list
        cmpw    $0, STATUS(%ebx)
        je      off_char_status
        cmpw    $1, STATUS(%ebx)
        je      on_char_status

on_char_status:
        movw    $0, STATUS(%ebx)
        movzwl  LOCATION(%ebx), %eax
        shll    $1,%eax        
        movb    OFF_CHAR(%ebx), %cl
        call    mp1_poke
        movw    OFF_LENGTH(%ebx),%ax
        movw    %ax, COUNTDOWN(%ebx)
        jmp     Next_list   

off_char_status:
        movw    $1, STATUS(%ebx) 
        movzwl  LOCATION(%ebx), %eax
        shll    $1,%eax
        movb    ON_CHAR(%ebx), %cl
        call    mp1_poke
        movw    ON_LENGTH(%ebx),%ax
        movw    %ax, COUNTDOWN(%ebx)
        jmp     Next_list

Next_list:
        cmpl    $0, NEXT(%ebx)
        je      over
        movl    NEXT(%ebx), %ebx
        jmp     Main

over:
        popl    %edi
        popl    %esi
        popl    %ebx
        leave
	ret

# ##############################################################################
# void mp1_ioctl(unsigned long arg,unsigned long cmd);
#
# Interface: Register-based arguments (not C-style)
#    Inputs: unsigned long cmd
#            unsigned long arg
#   Outputs: According the cmd value to jump into other functions
# Registers: Clobbers %eax

mp1_ioctl:
	xorl    %eax,%eax  # initialize the eax
        movl    8(%esp),%eax  # eax <- cmd
        cmpl    $3,%eax
        ja      invalid_cmd  # test whether the cmd is larger than 4
        cmpl    $0,%eax
        jb      invalid_cmd  # test whether the cmd is smaller than 0
        jmp     *jump_table(,%eax,4) # do the function according to the jump_table  

invalid_cmd:
        movl    $-1,%eax
        ret


# ##############################################################################
# void mp1_ioctl_add(unsigned long arg);
#
# Interface: Register-based arguments (not C-style)
#            Adds a blinking location
#    Inputs: %ebx - arg
#
#   Outputs: 0 for success; -1 for failure
mp1_ioctl_add:
 	pushl	%ebp			
	movl	%esp, %ebp		
	pushl	%ebx  # ebx = arg		
	pushl	%esi  
	pushl	%edi  # edi = malloc address

        movl    8(%ebp),%ebx
        cmpl    $0,%ebx
        je      go_error

        pushl   $STRUCT_SIZE
        call    mp1_malloc # eax = malloc address
        addl    $4,%esp
        cmpl    $0,%eax
        je      go_error

        movl    %eax,%edi
        pushl   $STRUCT_SIZE
        pushl   %ebx
        pushl   %edi
        call    mp1_copy_from_user
        cmpl    $0,%eax  #  check whether the copy is failed
        jne     go_error_free
        addl    $12,%esp  # now the esp pointer is at edi

        # check whether the location is inside or outside the valid range
        movzwl  LOCATION(%edi),%eax # eax here is used for temp 
        cmpl    $1999,%eax
        ja      go_error_free
        cmpl    $0,%eax
        jb      go_error_free

        # set the countdown field to be equal to the on_length field and set the status field to 1
        movw    ON_LENGTH(%edi), %ax	
	movw    %ax, COUNTDOWN(%edi) 		
        movw    $1, STATUS(%edi)

        # insert the struscure
        xorl    %eax,%eax  # initialize the eax
        movl    mp1_list_head,%eax
        movl    %eax,NEXT(%edi)
        movl    %edi,mp1_list_head

        # call mp1_poke
        movzwl LOCATION(%edi), %eax
        shll   $1,%eax
        movb   ON_CHAR(%edi),%cl  
        
        call  mp1_poke
        xorl    %eax,%eax  # initialize the eax
        popl    %edi
        popl    %esi
        popl    %ebx
        leave
	ret

go_error:
        movl    $-1,%eax
        popl    %edi
        popl    %esi
        popl    %ebx
        leave
	ret

go_error_free:
        pushl   %edi
        call    mp1_free
        movl    $-1,%eax
        popl    %edi
        popl    %esi
        popl    %ebx
        leave

# ##############################################################################
# int mp1_ioctl_find(unsigned long arg)
# Interface: find the list for an element that matches the location
# Inputs   : unsigned long arg 
# Outputs  : 0 for success; -1 for failure
mp1_ioctl_find:
 	pushl	%ebp			
	movl	%esp, %ebp		
	pushl	%ebx # ebx = arg 		
	pushl	%esi # esi = list_head 
	pushl	%edi 

        movl    8(%ebp),%ebx
        movl    mp1_list_head,%esi
        cmpl    $0,%ebx
        je      error_finish

        # check whether the list_head is null
        cmpl    $0,%esi
        jbe      error_finish
        # check whether it is out of range

find_Main:
        movw    LOCATION(%ebx),%dx
        cmpw    $1999,%dx
        ja      error_finish
        cmpw    $0,%dx
        jb      error_finish
        
        movw    LOCATION(%esi),%cx
        cmpw    %dx,%cx
        jne     Next_Node
        
        # If equal we call the copy_to_user
        pushl   $16
        pushl   %ebx
        pushl   %esi
        call    mp1_copy_from_user
        addl    $12,%esp
        cmpl    $0,%eax  #  check whether the copy is failed 
        jne     error_finish
        popl    %edi
        popl    %esi
        popl    %ebx
        leave
	ret

Next_Node:
        cmpl    $0,NEXT(%esi)
        je      error_finish
        movl    NEXT(%esi),%esi
        jmp     find_Main
        ret

error_finish:
        movl    $-1,%eax
        popl    %edi
        popl    %esi
        popl    %ebx
        leave
	ret

# ##############################################################################
# int mp1_ioctl_sync(unsigned long arg)
# Interface: synchronize the two locations on the screen
# Inputs   : unsigned long arg consists of two short(16bits) locations 
# Outputs  : 0 for success; -1 for failure

mp1_ioctl_sync:
	pushl	%ebp			
	movl	%esp, %ebp		
	pushl	%ebx # ebx first element
	pushl	%esi 
	pushl	%edi # edi second element
        
        movl    8(%ebp),%ebx
        movl    8(%ebp),%edi
        cmpl    $0,%edi
        je      sync_error 
        cmpl    $0,%ebx
        je      sync_error 
        
        shrl    $16,%ebx # now ebx has the upper 16 bits

        pushw   %bx
        call    traversal_search
        addl    $2,%esp
        cmpl    $-1,%eax
        je      sync_error 
        movl    %eax,%ebx # the first has been find

        pushw   %di
        call    traversal_search
        addl    $2,%esp
        cmpl    $-1,%eax
        je      sync_error 
        movl    %eax,%edi # the second has been find

        # do the copy using the stack
        xorl    %edx,%edx
        movw    ON_LENGTH(%ebx),%dx
        movw    %dx,ON_LENGTH(%edi)
        movw    OFF_LENGTH(%ebx),%dx
        movw    %dx,OFF_LENGTH(%edi)
        movw    COUNTDOWN(%ebx),%dx
        movw    %dx,COUNTDOWN(%edi)
        movw    STATUS(%ebx),%dx
        movw    %dx,STATUS(%edi)      

        cmpl    $1,STATUS(%edi)
        je      sync_on_char


        movb   OFF_CHAR(%edi),%cl
        movzwl LOCATION(%edi), %eax
        call   mp1_poke
        jmp    sync_done

sync_on_char:
        movb   ON_CHAR(%edi),%cl
        movzwl LOCATION(%edi), %eax
        call   mp1_poke

sync_done:
        popl    %edi
        popl    %esi
        popl    %ebx

        leave
        ret

sync_error:
        popl    %edi
        popl    %esi
        popl    %ebx
        movl    $-1,%eax
        leave
        ret

# ##############################################################################
# int mp1_ioctl_remove(unsigned long arg)
# Interface: remove the corresponding location
# Inputs   : unsigned long arg - integer of location
# Outputs  : 0 for success; -1 for failure
mp1_ioctl_remove:
	pushl	%ebp			
	movl	%esp, %ebp		
	pushl	%ebx 		
	pushl	%esi 
	pushl	%edi 

        movl    8(%ebp),%ebx
        movl    mp1_list_head,%edi
        cmpl    $0,%edi
        je      remove_error
        cmpl    $1999,%ebx
        jg      remove_error
        cmpl    $0,%ebx
        jl      remove_error
        pushw   %bx

        call    traversal_search
        addl    $2,%esp
        cmpl    $0,%eax
        je      remove_error

        cmpl    %edi,%eax # check whether its the head node
        jne     node_travel
        cmpl    $0,NEXT(%edi)# check whether is the end node
        je      remove_error
        movl    NEXT(%edi),%edi
        movl    %edi,mp1_list_head
        jmp     node_free  
node_travel:
        cmpl    $0,NEXT(%edi) # check whether is the end node
        je      remove_error
        cmpl    NEXT(%edi),%eax
        je      node_remove
        movl    NEXT(%edi),%edi
        jmp     node_travel

node_remove:
        movl    NEXT(%eax),%esi
        movl    %esi,NEXT(%edi)

node_free:
        pushl   %eax
        call    mp1_free
        addl    $4,%esp
        movl    $0,%eax
        popl    %edi
        popl    %esi
        popl    %ebx
        leave
        ret

remove_error:
        movl    $-1,%eax
        popl    %edi
        popl    %esi
        popl    %ebx
        leave
        ret






# ###################################################################################
# Helper Function
traversal_search:
        pushl	%ebp			
	movl	%esp, %ebp		
	pushl	%ebx 
	pushl	%esi 
	pushl	%edi 

        movl    mp1_list_head,%ebx # ebx = list_head
        movw    8(%ebp),%di 

Main_Loop:
        cmpl    $0,%ebx
        je      Search_error
        cmpw    LOCATION(%ebx),%di
        je      Finish
        cmpl    $0,NEXT(%ebx)
        je      Search_error
        movl    NEXT(%ebx),%ebx
        jmp     Main_Loop

Finish:
        movl    %ebx,%eax # the result is stored in the eax
        popl    %edi
        popl    %esi
        popl    %ebx
        leave
        ret

Search_error:
        popl    %edi
        popl    %esi
        popl    %ebx
        leave
        ret

# jump table for mp1_ioct1
jump_table:
.long mp1_ioctl_add,mp1_ioctl_remove,mp1_ioctl_find,mp1_ioctl_sync

.end
