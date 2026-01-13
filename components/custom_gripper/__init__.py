# __init__.py
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome import core

from esphome.const import CONF_ID

custom_gripper_ns = cg.esphome_ns.namespace('custom_gripper')
CustomGripper = custom_gripper_ns.class_('custom_gripper', cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.declare_id(CustomGripper),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
