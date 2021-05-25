use std::convert::TryInto;
use std::ffi::{CString, NulError};
use thiserror::Error;

pub struct Node {
    node: rcl_sys::rcl_node_t,
}

#[derive(Error, Debug, PartialEq)]
pub enum NodeNameValidationError {
    #[error("Node name cannot be represented as a C-style string")]
    InvalidCString(#[from] NulError),
    #[error("Node name is an empty String")]
    EmptyString,
    #[error("Node name contains an unallowed character at position `{0}`")]
    ContainsUnallowedCharacters(usize),
    #[error("Node name starts with a number")]
    StartsWithNumber,
    #[error("Node name is too long")]
    IsTooLong,
}

#[derive(Error, Debug, PartialEq)]
pub enum NodeNamespaceValidationError {
    #[error("Node namespace cannot be represented as a C-style string")]
    InvalidCString(#[from] NulError),
    #[error("Node namespace is an empty String")]
    EmptyString,
    #[error("Node namespace contains an unallowed character at position `{0}`")]
    ContainsUnallowedCharacters(usize),
    #[error("Node namespace starts with a number")]
    StartsWithNumber,
    #[error("Node namespace is too long")]
    IsTooLong,
    #[error("Node namespace is not absolute. It should start with a `/`.")]
    NotAbsolute,
    #[error("Node namespace ends with a forward slash")]
    EndsWithForwardSlash,
    #[error("Node namespace contains repeated forward slashes at position `{0}`")]
    ContainsRepeatedForwardSlash(usize),
}

impl Node {
    /**
     * Validate whether a node name is valid. Node names must follow these rules:
     * - must not be an empty string
     * - must only contain alphanumeric characters and underscores (a-z|A-Z|0-9|_)
     * - must not start with a number
     */
    pub fn validate_node_name<N: Into<String>>(name: N) -> Result<(), NodeNameValidationError> {
        let name = CString::new(name.into())?;
        let c_string = &name;

        let mut validation_result = 0;
        let mut invalid_index = 0;

        // Safety: all pointers and references have longer lifetimes than the
        // call of this function. This function only checks whether
        // the provided string follows the rules. Nothing is allocated.
        let return_value = unsafe {
            rcl_sys::rmw_validate_node_name(
                c_string.as_ptr(),
                &mut validation_result,
                &mut invalid_index,
            )
        };

        // If the return_value == OK then the check has succesfully been called.
        // However, this does not mean that the string is actually valid. This has to
        // be checked based on the `validation_result` variable.
        if return_value != rcl_sys::RMW_RET_OK.try_into().unwrap() {
            panic!("The rmw_validate_node_name function has not been called properly. This should never happen.")
        }

        // Map the possible validation_result values to Rust error types
        match validation_result as u32 {
            rcl_sys::RMW_NODE_NAME_VALID => Ok(()),
            rcl_sys::RMW_NODE_NAME_INVALID_IS_EMPTY_STRING => {
                Err(NodeNameValidationError::EmptyString)
            }
            rcl_sys::RMW_NODE_NAME_INVALID_CONTAINS_UNALLOWED_CHARACTERS => Err(
                NodeNameValidationError::ContainsUnallowedCharacters(invalid_index),
            ),
            rcl_sys::RMW_NODE_NAME_INVALID_STARTS_WITH_NUMBER => {
                Err(NodeNameValidationError::StartsWithNumber)
            }
            rcl_sys::RMW_NODE_NAME_INVALID_TOO_LONG => Err(NodeNameValidationError::IsTooLong),
            _ => panic!("Unknown error in validating node name"),
        }
    }

    /**
     * Validate whether a node namespace is valid. Node namespaces must follow these rules
     * defined in the [ROS2 Design Guide](https://design.ros2.org/articles/topic_and_service_names.html)
     */
    pub fn validate_node_namespace<N: Into<String>>(
        namespace: N,
    ) -> Result<(), NodeNamespaceValidationError> {
        let namespace = CString::new(namespace.into())?;
        let c_string = &namespace;

        let mut validation_result = 0;
        let mut invalid_index = 0;

        // Safety: all pointers and references have longer lifetimes than the
        // call of this function. This function only checks whether
        // the provided string follows the rules. Nothing is allocated.
        let return_value = unsafe {
            rcl_sys::rmw_validate_namespace(
                c_string.as_ptr(),
                &mut validation_result,
                &mut invalid_index,
            )
        };

        if return_value != rcl_sys::RMW_RET_OK.try_into().unwrap() {
            panic!("The rmw_validate_namespace function has not been called properly. This should never happen.")
        }

        /*
         *
         */
        // Map the possible validation_result values to Rust error types
        match validation_result as u32 {
            rcl_sys::RMW_NAMESPACE_VALID => Ok(()),
            rcl_sys::RMW_NAMESPACE_INVALID_IS_EMPTY_STRING => {
                Err(NodeNamespaceValidationError::EmptyString)
            }
            rcl_sys::RMW_NAMESPACE_INVALID_CONTAINS_UNALLOWED_CHARACTERS => Err(
                NodeNamespaceValidationError::ContainsUnallowedCharacters(invalid_index),
            ),
            rcl_sys::RMW_NAMESPACE_INVALID_NAME_TOKEN_STARTS_WITH_NUMBER => {
                Err(NodeNamespaceValidationError::StartsWithNumber)
            }
            rcl_sys::RMW_NAMESPACE_INVALID_TOO_LONG => Err(NodeNamespaceValidationError::IsTooLong),
            rcl_sys::RMW_NAMESPACE_INVALID_NOT_ABSOLUTE => {
                Err(NodeNamespaceValidationError::NotAbsolute)
            }
            rcl_sys::RMW_NAMESPACE_INVALID_ENDS_WITH_FORWARD_SLASH => {
                Err(NodeNamespaceValidationError::EndsWithForwardSlash)
            }
            rcl_sys::RMW_NAMESPACE_INVALID_CONTAINS_REPEATED_FORWARD_SLASH => Err(
                NodeNamespaceValidationError::ContainsRepeatedForwardSlash(invalid_index),
            ),
            _ => panic!("Unknown error in validating node namespace"),
        }
    }
}

pub struct NodeBuilder {
    name: Option<String>,
    namespace: String,
    options: rcl_sys::rcl_node_options_t,
}

impl Default for NodeBuilder {
    fn default() -> Self {
        Self {
            name: None,
            namespace: String::new(),
            options: unsafe { rcl_sys::rcl_node_get_default_options() },
        }
    }
}

impl NodeBuilder {
    /// Set the name of the node. This function will panic if the name does not
    /// pass [`Node::validate_node_name`].
    pub fn name<N: Into<String>>(mut self, name: N) -> NodeBuilder {
        let name = name.into();
        Node::validate_node_name(&name).expect("Node name should follow certain naming rules");
        self.name = Some(name);
        self
    }

    /// Set the namespace of the node. This function will panic if the namespace does not pass
    /// [`Node::validate_node_namespace`].
    pub fn namespace<N: Into<String>>(mut self, namespace: N) -> NodeBuilder {
        let namespace = namespace.into();
        Node::validate_node_namespace(&namespace)
            .expect("Node namespace should follow certain naming rules");
        self.namespace = namespace;
        self
    }

    /// Flag to enable the /rosout topic for this node. Enabled by default.
    pub fn enable_rosout(mut self, enable: bool) -> NodeBuilder {
        self.options.enable_rosout = enable;
        self
    }

    fn build(self) -> Node {
        todo!()
        /*
            rcl_context_t context = rcl_get_zero_initialized_context();
            //
            // ... initialize the context with rcl_init()
            rcl_node_t node = rcl_get_zero_initialized_node();
            rcl_node_options_t node_ops = rcl_node_get_default_options();
            // ... node options customization
            rcl_ret_t ret = rcl_node_init(&node, "node_name", "/node_ns", &context, &node_ops);
            // ... error handling and then use the node, but eventually deinitialize it:
            ret = rcl_node_fini(&node);
            // ... error handling for rcl_node_fini()
        */
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_node_name() {
        assert_eq!(Node::validate_node_name("random_node_name"), Ok(()))
    }

    #[test]
    fn test_invalid_node_name_empty_string() {
        assert_eq!(
            Node::validate_node_name(""),
            Err(NodeNameValidationError::EmptyString)
        )
    }

    #[test]
    fn test_invalid_node_name_unallowed_characters() {
        assert_eq!(
            Node::validate_node_name("node_name+something"),
            Err(NodeNameValidationError::ContainsUnallowedCharacters(9))
        )
    }

    #[test]
    fn test_invalid_node_name_starts_with_number() {
        assert_eq!(
            Node::validate_node_name("0_nodename"),
            Err(NodeNameValidationError::StartsWithNumber)
        )
    }

    #[test]
    fn test_invalid_node_name_too_long() {
        // The maximum length is arbitrarily defined by rmw
        let bytes = ['A' as u8].repeat(1000);
        let string = String::from_utf8_lossy(&bytes);
        assert_eq!(
            Node::validate_node_name(string),
            Err(NodeNameValidationError::IsTooLong)
        )
    }

    #[test]
    fn test_invalid_node_name_invalid_c_string() {
        // The maximum length is arbitrarily defined by rmw
        let bytes: Vec<u8> = vec![65, 66, 0, 65, 66];
        let string = String::from_utf8_lossy(&bytes);
        if let Err(NodeNameValidationError::InvalidCString(_)) = Node::validate_node_name(string) {
            assert!(true)
        } else {
            assert!(false)
        }
    }

    #[test]
    #[should_panic]
    fn test_invalid_node_name_in_node_builder() {
        NodeBuilder::default().name("Invalid+Name");
    }

    #[test]
    fn test_valid_node_namespace() {
        assert_eq!(Node::validate_node_namespace("/namespace"), Ok(()))
    }

    #[test]
    fn test_invalid_node_namespace_empty_string() {
        assert_eq!(
            Node::validate_node_namespace(""),
            Err(NodeNamespaceValidationError::EmptyString)
        )
    }

    #[test]
    fn test_invalid_node_namespace_not_absolute() {
        assert_eq!(
            Node::validate_node_namespace("just_a_namespace"),
            Err(NodeNamespaceValidationError::NotAbsolute)
        )
    }

    #[test]
    fn test_invalid_node_namespace_ends_with_forward_slash() {
        assert_eq!(
            Node::validate_node_namespace("/just_a_namespace/"),
            Err(NodeNamespaceValidationError::EndsWithForwardSlash)
        )
    }

    #[test]
    fn test_invalid_node_namespace_contains_unallowed_characters() {
        assert_eq!(
            Node::validate_node_namespace("/just-a-namespace"),
            Err(NodeNamespaceValidationError::ContainsUnallowedCharacters(5))
        )
    }

    #[test]
    fn test_invalid_node_namespace_contains_repeated_forwards_slash() {
        assert_eq!(
            Node::validate_node_namespace("/just_a_namespace//second_level"),
            Err(NodeNamespaceValidationError::ContainsRepeatedForwardSlash(
                18
            ))
        )
    }

    #[test]
    fn test_invalid_node_namespace_starts_with_number() {
        assert_eq!(
            Node::validate_node_namespace("/21_namespace"),
            Err(NodeNamespaceValidationError::StartsWithNumber)
        )
    }

    #[test]
    fn test_invalid_node_namespace_too_long() {
        // The maximum length is arbitrarily defined by rmw
        let bytes = ['/' as u8, 'A' as u8].repeat(500);
        let string = String::from_utf8_lossy(&bytes);
        assert_eq!(
            Node::validate_node_namespace(string),
            Err(NodeNamespaceValidationError::IsTooLong)
        )
    }

    #[test]
    fn test_invalid_nodespace_invalid_c_string() {
        // The maximum length is arbitrarily defined by rmw
        let bytes: Vec<u8> = vec![47, 65, 66, 0, 65, 66];
        let string = String::from_utf8_lossy(&bytes);
        if let Err(NodeNamespaceValidationError::InvalidCString(_)) =
            Node::validate_node_namespace(string)
        {
            assert!(true)
        } else {
            assert!(false)
        }
    }

    #[test]
    #[should_panic]
    fn test_invalid_node_namespace_in_node_builder() {
        NodeBuilder::default().namespace("invalid_namespace");
    }
}
